#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gemini Agent 단독 실행 런치 파일
에이전트 노드만 실행합니다 (개발/테스트용).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """런치 설정 생성"""
    
    pkg_name = 'my_gemini_robot'
    pkg_dir = get_package_share_directory(pkg_name)
    config_file = os.path.join(pkg_dir, 'config', 'gemini_params.yaml')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='설정 파일 경로'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file_path = LaunchConfiguration('config_file')
    
    agent_node = Node(
        package=pkg_name,
        executable='gemini_agent',
        name='gemini_agent',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        agent_node
    ])

