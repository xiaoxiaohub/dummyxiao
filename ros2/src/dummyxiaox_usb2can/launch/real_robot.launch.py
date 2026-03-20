#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dummy", package_name="dummyxiaox_moveit_config")
        .robot_description()
        .to_moveit_configs()
    )

    # ros2_control with mock hardware: receives trajectory commands from MoveIt
    # and publishes /joint_states at the correct timing. usb2can_node subscribes
    # to /joint_states and forwards the commands to real motors via CAN bus.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                    "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dummyxiaox_moveit_config_controller",
                    "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"start_state_max_bounds_error": 0.1},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dummyxiaox_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # USB serial bridge: subscribes to /joint_states and sends ASCII commands
    # to the REF Core Board via USB CDC serial (/dev/ttyACM0).
    # The core board handles CAN communication to CtrlStep motors internally.
    usb2can_node = Node(
        package="dummyxiaox_usb2can",
        executable="usb2can_node",
        name="usb2can_node",
        output="screen",
        parameters=[{
            "serial_port": "/dev/ttyACM0",
            "baudrate": 115200,
            "speed": 30.0,
            "command_mode": 2,
        }],
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rsp_node,
        move_group_node,
        rviz_node,
        usb2can_node,
    ])
