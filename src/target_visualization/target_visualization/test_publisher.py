#!/usr/bin/env python3
"""
测试可视化系统的脚本
发布模拟的图像和目标数据用于测试
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import math

# ROS2 消息类型
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # CV Bridge用于图像转换
        self.bridge = CvBridge()
        
        # 创建发布器
        self.image_publisher = self.create_publisher(
            Image,
            'image_topic',
            10
        )
        
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'visualization_targets',
            10
        )
        
        # 创建定时器
        self.image_timer = self.create_timer(0.1, self.publish_test_image)  # 10Hz
        self.marker_timer = self.create_timer(1.0, self.publish_test_markers)  # 1Hz
        
        # 初始化计数器
        self.frame_count = 0
        
        self.get_logger().info('测试发布器已启动')

    def create_test_image(self):
        """创建测试图像"""
        # 创建一个640x480的彩色图像
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 添加渐变背景
        for y in range(480):
            for x in range(640):
                image[y, x] = [int(x * 255 / 640), int(y * 255 / 480), 128]
        
        # 添加一些图案
        cv2.circle(image, (160, 120), 50, (255, 255, 255), 2)
        cv2.circle(image, (480, 240), 60, (255, 255, 255), 2)
        cv2.circle(image, (320, 360), 40, (255, 255, 255), 2)
        
        # 添加文本
        cv2.putText(image, f"Test Frame: {self.frame_count}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return image

    def publish_test_image(self):
        """发布测试图像"""
        try:
            # 创建测试图像
            cv_image = self.create_test_image()
            
            # 转换为ROS消息
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            # 发布图像
            self.image_publisher.publish(ros_image)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'图像发布错误: {str(e)}')

    def publish_test_markers(self):
        """发布测试标记"""
        try:
            marker_array = MarkerArray()
            
            # 创建动态变化的测试标记
            time_factor = time.time()
            
            # 创建多个测试标记
            markers_data = [
                {
                    'id': 1,
                    'category': 'circle',
                    'pos': [2.0 + math.sin(time_factor) * 0.5, 1.0, 10.0],
                    'radius': 0.5,
                    'color': [1.0, 0.0, 0.0, 1.0]  # 红色
                },
                {
                    'id': 2,
                    'category': 'H_type',
                    'pos': [4.0, -1.0 + math.cos(time_factor) * 0.3, 12.0],
                    'radius': 0.3,
                    'color': [0.0, 1.0, 0.0, 1.0]  # 绿色
                },
                {
                    'id': 3,
                    'category': 'circle',
                    'pos': [1.0, 2.0, 8.0 + math.sin(time_factor * 0.5) * 2],
                    'radius': 0.4,
                    'color': [0.0, 0.0, 1.0, 1.0]  # 蓝色
                }
            ]
            
            for marker_data in markers_data:
                marker = Marker()
                
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = marker_data['category']
                marker.id = marker_data['id']
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                # 设置位置
                marker.pose.position.x = marker_data['pos'][0]
                marker.pose.position.y = marker_data['pos'][1]
                marker.pose.position.z = marker_data['pos'][2]
                marker.pose.orientation.w = 1.0
                
                # 设置尺寸（直径）
                marker.scale.x = marker_data['radius'] * 2
                marker.scale.y = marker_data['radius'] * 2
                marker.scale.z = 0.1
                
                # 设置颜色
                marker.color.r = marker_data['color'][0]
                marker.color.g = marker_data['color'][1]
                marker.color.b = marker_data['color'][2]
                marker.color.a = marker_data['color'][3]
                
                # 设置生存时间
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 0
                
                marker_array.markers.append(marker)
            
            # 发布标记数组
            self.marker_publisher.publish(marker_array)
            
            self.get_logger().info(f'发布了 {len(markers_data)} 个测试标记')
            
        except Exception as e:
            self.get_logger().error(f'标记发布错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = TestPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
