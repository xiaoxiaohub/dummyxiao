#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

import rclpy
from rclpy.node import Node
import can
from typing import Dict
from threading import Lock
import struct
import json
import time
import math
from dummyx_interface.srv import InitUsb2Can
from dummyx_interface.srv import WriteUsb2Can
from dummyx_interface.srv import ReadUsb2Can
from sensor_msgs.msg import JointState

# CtrlStep CAN Command Definitions
# CAN ID format: (node_id << 7) | cmd   (4-bit node_id, 7-bit cmd)
CMD_ENABLE = 0x01
CMD_CALIBRATION = 0x02
CMD_SET_CURRENT = 0x03
CMD_SET_VELOCITY = 0x04
CMD_SET_POSITION = 0x05
CMD_SET_POSITION_WITH_TIME = 0x06
CMD_SET_POSITION_WITH_VEL_LIMIT = 0x07
CMD_SET_NODE_ID = 0x11
CMD_SET_CURRENT_LIMIT = 0x12
CMD_SET_VELOCITY_LIMIT = 0x13
CMD_SET_ACCELERATION = 0x14
CMD_APPLY_HOME = 0x15
CMD_SET_ENABLE_ON_BOOT = 0x16
CMD_SET_DCE_KP = 0x17
CMD_SET_DCE_KV = 0x18
CMD_SET_DCE_KI = 0x19
CMD_SET_DCE_KD = 0x1A
CMD_SET_STALL_PROTECT = 0x1B
CMD_GET_CURRENT = 0x21
CMD_GET_VELOCITY = 0x22
CMD_GET_POSITION = 0x23
CMD_GET_OFFSET = 0x24
CMD_GET_TEMPERATURE = 0x25
CMD_ENABLE_TEMP_WATCH = 0x7D
CMD_ERASE_CONFIGS = 0x7E
CMD_REBOOT = 0x7F

DEFAULT_VELOCITY_LIMIT = 10.0  # rev/s
DEFAULT_CURRENT_LIMIT = 2.0    # Amps


class Motor:
    def __init__(self, bus: can.Bus, node_id: int, reduction: float,
                 inverse: bool = False):
        self.bus = bus
        self.node_id = node_id
        self.reduction = reduction
        self.inverse = inverse
        self.position = 0.0
        self.finished = False
        self.enabled = False
        self.lock = Lock()
        self.last_sent_position = None

    def build_can_id(self, cmd: int) -> int:
        return (self.node_id << 7) | cmd

    def _send(self, cmd: int, data: bytes = b''):
        payload = data.ljust(8, b'\x00')
        msg = can.Message(
            arbitration_id=self.build_can_id(cmd),
            data=payload,
            is_extended_id=False
        )
        with self.lock:
            self.bus.send(msg)

    def enable(self):
        self._send(CMD_ENABLE, struct.pack('<I', 1))
        self.enabled = True

    def disable(self):
        self._send(CMD_ENABLE, struct.pack('<I', 0))
        self.enabled = False

    def send_position(self, position: float, velocity_limit: float):
        """Send position with velocity limit (cmd 0x07).

        Position is in motor revolutions (angle_deg / 360 * reduction).
        Velocity limit is in rev/s at the motor shaft.
        Direction inversion is applied here.
        """
        if self.inverse:
            position = -position
            velocity_limit = abs(velocity_limit)

        if (self.last_sent_position is not None and
                abs(position - self.last_sent_position) < 0.0001):
            return

        data = struct.pack('<ff', position, velocity_limit)
        self._send(CMD_SET_POSITION_WITH_VEL_LIMIT, data)
        self.last_sent_position = position

    def send_position_simple(self, position: float, request_ack: bool = True):
        """Send position setpoint (cmd 0x05).

        Position is in motor revolutions.
        """
        if self.inverse:
            position = -position

        data = struct.pack('<f', position) + struct.pack('B', 1 if request_ack else 0)
        self._send(CMD_SET_POSITION, data)

    def request_position(self):
        """Request position feedback (cmd 0x23)."""
        self._send(CMD_GET_POSITION)

    def set_velocity_limit(self, limit: float, save: bool = False):
        data = struct.pack('<f', limit) + struct.pack('B', 1 if save else 0)
        self._send(CMD_SET_VELOCITY_LIMIT, data)

    def set_current_limit(self, limit: float, save: bool = False):
        data = struct.pack('<f', limit) + struct.pack('B', 1 if save else 0)
        self._send(CMD_SET_CURRENT_LIMIT, data)

    def set_acceleration(self, acc: float, save: bool = False):
        data = struct.pack('<f', acc) + struct.pack('B', 1 if save else 0)
        self._send(CMD_SET_ACCELERATION, data)

    def update_position_callback(self, msg: can.Message):
        """Parse cmd 0x23 response: float position (bytes 0-3) + byte finished (byte 4)."""
        if len(msg.data) >= 5:
            with self.lock:
                raw_pos = struct.unpack('<f', msg.data[0:4])[0]
                self.finished = bool(msg.data[4])
                if self.inverse:
                    raw_pos = -raw_pos
                self.position = raw_pos

    def get_angle_degrees(self) -> float:
        """Convert stored position (motor revolutions) to joint angle in degrees."""
        with self.lock:
            return self.position / self.reduction * 360.0


class MotorController:
    def __init__(self, interface: str, channel: str,
                 motor_config: Dict[int, dict]):
        self.bus = can.Bus(interface=interface, channel=channel,
                           receive_own_messages=True)
        self.motors: Dict[int, Motor] = {}

        for node_id, cfg in motor_config.items():
            reduction = cfg.get('reduction', 30.0)
            inverse = cfg.get('inverse', False)
            self.motors[node_id] = Motor(
                self.bus, node_id, reduction, inverse
            )

        self.notifier = can.Notifier(self.bus, [self.on_message_received])

    def enable_all_motors(self):
        for motor in self.motors.values():
            motor.enable()
            time.sleep(0.02)

    def disable_all_motors(self):
        for motor in self.motors.values():
            motor.disable()
            time.sleep(0.02)

    def on_message_received(self, msg):
        node_id = (msg.arbitration_id >> 7) & 0x0F
        cmd = msg.arbitration_id & 0x7F

        if node_id in self.motors:
            if cmd == CMD_GET_POSITION:
                self.motors[node_id].update_position_callback(msg)

    def request_all_positions(self):
        for motor in self.motors.values():
            motor.request_position()
            time.sleep(0.005)

    def shutdown(self):
        self.notifier.stop()
        self.bus.shutdown()


DEFAULT_MOTOR_CONFIG = json.dumps({
    "1": {"reduction": 30, "inverse": True},
    "2": {"reduction": 30, "inverse": False},
    "3": {"reduction": 30, "inverse": True},
    "4": {"reduction": 24, "inverse": False},
    "5": {"reduction": 30, "inverse": True},
    "6": {"reduction": 50, "inverse": True},
})


class USB2CANNode(Node):
    def __init__(self):
        super().__init__('usb2can_node')

        self.declare_parameter('interface', 'socketcan')
        self.declare_parameter('channel', 'can0')
        self.declare_parameter('motor_config_json', DEFAULT_MOTOR_CONFIG)
        self.declare_parameter('joint_names',
                               ['Joint1', 'Joint2', 'Joint3',
                                'Joint4', 'Joint5', 'Joint6'])
        self.declare_parameter('velocity_limit', DEFAULT_VELOCITY_LIMIT)
        self.declare_parameter('current_limit', DEFAULT_CURRENT_LIMIT)

        self.init_srv = self.create_service(
            InitUsb2Can, 'init_usb2can', self.init_usb2can_callback)
        self.write_srv = self.create_service(
            WriteUsb2Can, 'write_usb2can', self.write_usb2can_callback)
        self.read_srv = self.create_service(
            ReadUsb2Can, 'read_usb2can', self.read_usb2can_callback)

        interface = self.get_parameter('interface').value
        channel = self.get_parameter('channel').value
        motor_config_json = self.get_parameter('motor_config_json').value
        self.joint_names = self.get_parameter('joint_names').value
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.current_limit = self.get_parameter('current_limit').value

        try:
            raw = json.loads(motor_config_json)
            motor_config = {}
            for k, v in raw.items():
                motor_config[int(k)] = {
                    'reduction': float(v.get('reduction', 30.0)),
                    'inverse': bool(v.get('inverse', False)),
                }
        except (json.JSONDecodeError, ValueError, AttributeError) as e:
            self.get_logger().error(f"Invalid motor_config_json: {e}")
            motor_config = {
                1: {'reduction': 30.0, 'inverse': True},
                2: {'reduction': 30.0, 'inverse': False},
                3: {'reduction': 30.0, 'inverse': True},
                4: {'reduction': 24.0, 'inverse': False},
                5: {'reduction': 30.0, 'inverse': True},
                6: {'reduction': 50.0, 'inverse': True},
            }

        self.get_logger().info(
            f"Initializing USB2CAN: interface={interface}, channel={channel}")
        self.get_logger().info(f"Motor configuration: {motor_config}")

        try:
            self.controller = MotorController(
                interface=interface,
                channel=channel,
                motor_config=motor_config,
            )
            self.get_logger().info("CAN bus initialized successfully")

            for motor in self.controller.motors.values():
                motor.set_current_limit(self.current_limit)
                time.sleep(0.01)
                motor.set_velocity_limit(self.velocity_limit)
                time.sleep(0.01)

            self.controller.enable_all_motors()
            self.get_logger().info("All motors enabled")

            self.joint_state_sub = self.create_subscription(
                JointState, 'joint_states',
                self.joint_state_callback, 10)
            self.get_logger().info("Subscribed to joint_states topic")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize USB2CAN: {e}")
            raise

    def joint_state_callback(self, msg: JointState):
        if not all(name in msg.name for name in self.joint_names):
            return

        for i, joint_name in enumerate(self.joint_names):
            try:
                joint_idx = msg.name.index(joint_name)
                joint_pos_rad = msg.position[joint_idx]
                joint_pos_deg = math.degrees(joint_pos_rad)

                motor_id = i + 1
                if motor_id not in self.controller.motors:
                    continue

                motor = self.controller.motors[motor_id]
                motor_revs = joint_pos_deg / 360.0 * motor.reduction

                vel_limit = self.velocity_limit
                if msg.velocity and joint_idx < len(msg.velocity):
                    joint_vel_rad = msg.velocity[joint_idx]
                    vel_limit = abs(joint_vel_rad / (2 * math.pi)
                                    * motor.reduction)
                    vel_limit = max(vel_limit, 0.5)

                motor.send_position(motor_revs, vel_limit)

            except (ValueError, IndexError) as e:
                self.get_logger().error(
                    f"Error processing joint {joint_name}: {e}")

    def init_usb2can_callback(self, request, response):
        self.get_logger().info(f"init_usb2can: action={request.action}")
        try:
            if request.action == 'stop':
                self.controller.disable_all_motors()
            elif request.action == 'start':
                self.controller.enable_all_motors()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"init_usb2can failed: {e}")
            response.success = False
        return response

    def write_usb2can_callback(self, request, response):
        try:
            pos_cmds = list(request.pos_commands)
            vel_cmds = list(request.vel_commands)

            for i, motor_id in enumerate(sorted(self.controller.motors.keys())):
                if i >= len(pos_cmds):
                    break
                motor = self.controller.motors[motor_id]
                pos_deg = pos_cmds[i]
                motor_revs = pos_deg / 360.0 * motor.reduction

                vel = DEFAULT_VELOCITY_LIMIT
                if i < len(vel_cmds) and vel_cmds[i] > 0:
                    vel = vel_cmds[i] / 360.0 * motor.reduction
                motor.send_position(motor_revs, vel)

            response.success = True
        except Exception as e:
            self.get_logger().error(f"write_usb2can failed: {e}")
            response.success = False
        return response

    def read_usb2can_callback(self, request, response):
        try:
            self.controller.request_all_positions()
            time.sleep(0.05)

            positions = []
            velocities = []
            for motor_id in sorted(self.controller.motors.keys()):
                motor = self.controller.motors[motor_id]
                positions.append(motor.get_angle_degrees())
                velocities.append(0.0)

            response.pos_commands = positions
            response.vel_commands = velocities
            response.success = True
        except Exception as e:
            self.get_logger().error(f"read_usb2can failed: {e}")
            response.pos_commands = [0.0] * 6
            response.vel_commands = [0.0] * 6
            response.success = False
        return response

    def on_shutdown(self):
        self.get_logger().info("Shutting down USB2CAN controller")
        if hasattr(self, 'controller'):
            self.controller.disable_all_motors()
            self.controller.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = USB2CANNode()
        rclpy.spin(node)
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Error in USB2CAN node: {e}")
        else:
            print(f"Error creating USB2CAN node: {e}")
    finally:
        if node is not None:
            node.on_shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
