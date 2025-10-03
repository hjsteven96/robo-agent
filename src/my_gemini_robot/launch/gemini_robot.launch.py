#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Gemini Robot 통합 런치 파일
모든 노드를 한 번에 실행합니다.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """런치 설정 생성"""
    
    # 패키지 경로
    pkg_name = 'my_gemini_robot'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # 설정 파일 경로
    config_file = os.path.join(pkg_dir, 'config', 'gemini_params.yaml')
    
    # Launch Arguments
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
    
    # 노드 설정
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file_path = LaunchConfiguration('config_file')
    
    # Gemini Planner 노드
    planner_node = Node(
        package=pkg_name,
        executable='gemini_planner',
        name='gemini_planner',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True
    )
    
    # Gemini Vision 노드
    vision_node = Node(
        package=pkg_name,
        executable='gemini_vision',
        name='gemini_vision',
        output='screen',
        parameters=[
            config_file_path,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True
    )
    
    # Gemini Agent 노드
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
    
    # 시작 로그
    start_log = LogInfo(
        msg='Gemini Robot 시스템 시작 중...'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        start_log,
        planner_node,
        vision_node,
        agent_node
    ])

