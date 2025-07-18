#!/usr/bin/env python3
"""
目标可视化系统启动脚本
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'camera_topic',
            default_value='image_topic',
            description='输入相机图像topic'
        ),
        
        DeclareLaunchArgument(
            'output_topic', 
            default_value='image_overlay_output',
            description='输出叠加图像topic'
        ),
        
        DeclareLaunchArgument(
            'targets_topic',
            default_value='visualization_targets',
            description='目标标记消息topic'
        ),
        
        DeclareLaunchArgument(
            'enable_test_publisher',
            default_value='true',
            description='是否启用测试数据发布器'
        ),
        
        # 日志信息
        LogInfo(msg="启动目标可视化叠加器..."),
        
        # 测试数据发布器
        Node(
            package='target_visualization',
            executable='test_publisher',
            name='test_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_test_publisher'))
        ),
        
        # 可视化节点
        Node(
            package='target_visualization',
            executable='target_overlay_visualizer',
            name='target_overlay_visualizer',
            output='screen',
            remappings=[
                ('image_topic', LaunchConfiguration('camera_topic')),
                ('image_overlay_output', LaunchConfiguration('output_topic')),
                ('visualization_targets', LaunchConfiguration('targets_topic')),
            ]
        ),
    ])
