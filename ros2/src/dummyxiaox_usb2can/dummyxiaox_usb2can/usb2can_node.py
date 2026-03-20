#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

import rclpy
from rclpy.node import Node
import serial
from threading import Lock
import time
import math
from dummyxiaox_interface.srv import InitUsb2Can
from dummyxiaox_interface.srv import WriteUsb2Can
from dummyxiaox_interface.srv import ReadUsb2Can
from sensor_msgs.msg import JointState

DEFAULT_SPEED = 30.0  # 0-100 scale, firmware default is 30 deg/s


class SerialController:
    """Communicates with the REF Core Board (STM32F405) via USB CDC serial.

    The REF Core Board handles CAN communication to CtrlStep motors internally.
    We send ASCII commands over serial; the board translates them to CAN frames.
    """

    def __init__(self, port: str, baudrate: int, timeout: float = 0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.lock = Lock()
        time.sleep(0.5)
        self._flush()

    def _flush(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send_command(self, cmd: str) -> str:
        """Send an ASCII command and return the response line."""
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + '\n').encode('ascii'))
            try:
                response = self.ser.readline().decode('ascii', errors='replace').strip()
            except serial.SerialTimeoutException:
                response = ''
            return response

    def send_command_no_wait(self, cmd: str):
        """Send an ASCII command without waiting for response."""
        with self.lock:
            self.ser.write((cmd + '\n').encode('ascii'))

    def enable(self):
        return self.send_command('!START')

    def disable(self):
        return self.send_command('!DISABLE')

    def emergency_stop(self):
        return self.send_command('!STOP')

    def home(self):
        return self.send_command('!HOME')

    def set_command_mode(self, mode: int):
        return self.send_command(f'#CMDMODE {mode}')

    def move_joints(self, j1: float, j2: float, j3: float,
                    j4: float, j5: float, j6: float,
                    speed: float = None):
        """Send joint position command (degrees).

        Format: >j1,j2,j3,j4,j5,j6[,speed]
        In INTERRUPTABLE mode (2), this immediately updates the target
        without waiting for motion to complete.
        """
        if speed is not None:
            cmd = f'>{j1:.2f},{j2:.2f},{j3:.2f},{j4:.2f},{j5:.2f},{j6:.2f},{speed:.1f}'
        else:
            cmd = f'>{j1:.2f},{j2:.2f},{j3:.2f},{j4:.2f},{j5:.2f},{j6:.2f}'
        self.send_command_no_wait(cmd)

    def get_joint_positions(self) -> list:
        """Query current joint positions. Returns list of 6 floats (degrees)."""
        response = self.send_command('#GETJPOS')
        if response.startswith('ok'):
            parts = response.split()
            if len(parts) >= 7:
                return [float(parts[i]) for i in range(1, 7)]
        return None

    def shutdown(self):
        if self.ser.is_open:
            self.ser.close()


class USB2CANNode(Node):
    def __init__(self):
        super().__init__('usb2can_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('joint_names',
                               ['Joint1', 'Joint2', 'Joint3',
                                'Joint4', 'Joint5', 'Joint6'])
        self.declare_parameter('speed', DEFAULT_SPEED)
        self.declare_parameter('command_mode', 2)

        self.init_srv = self.create_service(
            InitUsb2Can, 'init_usb2can', self.init_usb2can_callback)
        self.write_srv = self.create_service(
            WriteUsb2Can, 'write_usb2can', self.write_usb2can_callback)
        self.read_srv = self.create_service(
            ReadUsb2Can, 'read_usb2can', self.read_usb2can_callback)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.joint_names = self.get_parameter('joint_names').value
        self.speed = self.get_parameter('speed').value
        command_mode = self.get_parameter('command_mode').value

        self.get_logger().info(
            f"Initializing USB2CAN: port={serial_port}, baudrate={baudrate}")

        try:
            self.controller = SerialController(
                port=serial_port,
                baudrate=baudrate,
            )
            self.get_logger().info("Serial connection established")

            resp = self.controller.set_command_mode(command_mode)
            self.get_logger().info(
                f"Command mode set to {command_mode}: {resp}")

            resp = self.controller.enable()
            self.get_logger().info(f"Motors enabled: {resp}")

            self.last_positions = [None] * 6
            self.last_send_time = 0.0
            self.min_send_interval = 0.02  # 50 Hz max to avoid flooding

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

        now = time.monotonic()
        if now - self.last_send_time < self.min_send_interval:
            return

        joints_deg = [0.0] * 6
        for i, joint_name in enumerate(self.joint_names):
            try:
                joint_idx = msg.name.index(joint_name)
                joints_deg[i] = math.degrees(msg.position[joint_idx])
            except (ValueError, IndexError) as e:
                self.get_logger().error(
                    f"Error processing joint {joint_name}: {e}")
                return

        if self.last_positions[0] is not None:
            max_delta = max(abs(joints_deg[i] - self.last_positions[i])
                           for i in range(6))
            if max_delta < 0.01:
                return

        self.controller.move_joints(
            joints_deg[0], joints_deg[1], joints_deg[2],
            joints_deg[3], joints_deg[4], joints_deg[5],
            speed=self.speed)

        self.last_positions = joints_deg
        self.last_send_time = now

    def init_usb2can_callback(self, request, response):
        self.get_logger().info(f"init_usb2can: action={request.action}")
        try:
            if request.action == 'stop':
                self.controller.emergency_stop()
            elif request.action == 'start':
                self.controller.enable()
            elif request.action == 'disable':
                self.controller.disable()
            elif request.action == 'home':
                self.controller.home()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"init_usb2can failed: {e}")
            response.success = False
        return response

    def write_usb2can_callback(self, request, response):
        try:
            pos_cmds = list(request.pos_commands)
            vel_cmds = list(request.vel_commands)

            if len(pos_cmds) < 6:
                pos_cmds.extend([0.0] * (6 - len(pos_cmds)))

            speed = self.speed
            if vel_cmds and vel_cmds[0] > 0:
                speed = vel_cmds[0]

            self.controller.move_joints(
                pos_cmds[0], pos_cmds[1], pos_cmds[2],
                pos_cmds[3], pos_cmds[4], pos_cmds[5],
                speed=speed)

            response.success = True
        except Exception as e:
            self.get_logger().error(f"write_usb2can failed: {e}")
            response.success = False
        return response

    def read_usb2can_callback(self, request, response):
        try:
            positions = self.controller.get_joint_positions()
            if positions is not None:
                response.pos_commands = positions
                response.vel_commands = [0.0] * 6
                response.success = True
            else:
                self.get_logger().warn("Failed to read joint positions")
                response.pos_commands = [0.0] * 6
                response.vel_commands = [0.0] * 6
                response.success = False
        except Exception as e:
            self.get_logger().error(f"read_usb2can failed: {e}")
            response.pos_commands = [0.0] * 6
            response.vel_commands = [0.0] * 6
            response.success = False
        return response

    def on_shutdown(self):
        self.get_logger().info("Shutting down USB2CAN controller")
        if hasattr(self, 'controller'):
            self.controller.disable()
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
