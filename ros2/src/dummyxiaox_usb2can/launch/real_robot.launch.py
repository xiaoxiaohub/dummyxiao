#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load MoveIt Configuration
    # Declare Launch Argument
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_mode",
            default_value="passthrough",
            description="Control mode: passthrough or trap_traj",
        )
    )

    control_mode = LaunchConfiguration("control_mode")

    # 1. Load MoveIt Configuration
    # We pass the control_mode mapping to the robot_description loader
    moveit_config = (
        MoveItConfigsBuilder("dummy", package_name="dummyxiaox_moveit_config")
        .robot_description(mappings={"control_mode": control_mode})
        .to_moveit_configs()
    )

    # 2. ROS2 Control Node (The Real Hardware Interface)
    # This node loads the controller_manager and the hardware_interface plugin
    # configured in dummyx2.ros2_control.xacro (which we updated to use dummyx_hardware)
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

    # 3. Spawners for Controllers
    # Joint State Broadcaster (Publishes /joint_states from hardware)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Arm Controller (FollowJointTrajectory Action Server)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dummyxiaox_moveit_config_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 4. Robot State Publisher
    # Takes /joint_states and publishes TFs
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 5. Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"start_state_max_bounds_error": 0.1},
        ],
    )

    # 6. RViz
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

    # Define startup sequence to ensure controllers spawn after manager is up
    # However, just adding them to the list usually works as they wait for the service.
    # Delaying move_group slightly can help wait for TFs but standard practice is fine.

    return LaunchDescription(declared_arguments + [
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rsp_node,
        move_group_node,
        rviz_node,
    ])

