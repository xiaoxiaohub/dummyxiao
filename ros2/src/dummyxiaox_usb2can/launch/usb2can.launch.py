#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

from launch import LaunchDescription
from launch_ros.actions import Node

MOTOR_CONFIG = (
    '{"1":{"reduction":30,"inverse":true},'
    '"2":{"reduction":30,"inverse":false},'
    '"3":{"reduction":30,"inverse":true},'
    '"4":{"reduction":24,"inverse":false},'
    '"5":{"reduction":30,"inverse":true},'
    '"6":{"reduction":50,"inverse":true}}'
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummyxiaox_usb2can',
            executable='usb2can_node',
            name='usb2can_node',
            parameters=[{
                'interface': 'socketcan',
                'channel': 'can0',
                'motor_config_json': MOTOR_CONFIG,
            }]
        )
    ])
