#!/usr/bin/env python3
"""
目标可视化叠加器
订阅visualization_msgs/MarkerArray消息和图像，在图像上叠加圆形目标标记
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

# ROS2 消息类型
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

class TargetOverlayVisualizer(Node):
    def __init__(self):
        super().__init__('target_overlay_visualizer')
        
        # CV Bridge用于图像转换
        self.bridge = CvBridge()
        
        # 存储当前的目标信息
        self.current_targets = []
        
        # 相机参数（需要根据实际情况调整）
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = 1280
        self.image_height = 720
        
        # 创建订阅器
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            'visualization_targets',  # 匹配C++代码中的topic名称
            self.marker_callback,
            10
        )
        
        self.image_subscriber = self.create_subscription(
            Image,
            'image_topic',  # 输入图像topic
            self.image_callback,
            10
        )
        
        # # 创建发布器
        # self.overlay_publisher = self.create_publisher(
        #     Image,
        #     'image_overlay_output',  # 叠加后的图像topic
        #     10
        # )
        
        # 图像显示窗口名称
        self.window_name = 'Target Overlay Visualization'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('目标叠加可视化器已启动')

    def marker_callback(self, msg: MarkerArray):
        """处理接收到的标记数组消息"""
        self.current_targets = []
        
        for marker in msg.markers:
            if marker.type == Marker.CYLINDER and marker.action == Marker.ADD:
                try:
                    target_info = {
                        'id': int(marker.id),
                        'category': str(marker.ns),
                        'x': float(marker.pose.position.x),
                        'y': float(marker.pose.position.y),
                        'z': 1.0,  # 假设z轴为1.0，实际应用中可能需要根据具体情况调整
                        'radius': max(1.0, float(marker.scale.x / 2.0)),  # 从直径转换为半径，确保最小为1
                        'color': {
                            'r': max(0, min(255, int(marker.color.r * 255))),
                            'g': max(0, min(255, int(marker.color.g * 255))),
                            'b': max(0, min(255, int(marker.color.b * 255))),
                            'a': float(marker.color.a)
                        }
                    }
                    self.current_targets.append(target_info)
                except Exception as e:
                    self.get_logger().error(f'处理标记 {marker.id} 时出错: {str(e)}')
                    continue
        
        self.get_logger().debug(f'收到 {len(self.current_targets)} 个目标标记')

    # def world_to_image(self, world_x, world_y, world_z):
    #     """
    #     将世界坐标转换为图像像素坐标
    #     这里需要根据实际的相机标定参数和坐标系变换来实现
    #     目前使用简化的投影模型
    #     """
    #     # 简化的透视投影（需要根据实际情况调整）
    #     # 假设相机位于原点，朝向z轴正方向
        
    #     if world_z <= 0:
    #         return None, None
        
    #     # 简化的针孔相机模型
    #     focal_length = 500  # 像素单位的焦距，需要标定
        
    #     # 投影到图像平面
    #     image_x = int((world_x / world_z) * focal_length + self.image_width / 2)
    #     image_y = int((world_y / world_z) * focal_length + self.image_height / 2)
        
    #     # 检查是否在图像范围内
    #     if 0 <= image_x < self.image_width and 0 <= image_y < self.image_height:
    #         return image_x, image_y
    #     else:
    #         return None, None

    # def calculate_pixel_radius(self, world_radius, world_z):
    #     """计算目标在图像中的像素半径"""
    #     if world_z <= 0:
    #         return 0
        
    #     focal_length = 500  # 与world_to_image中的一致
    #     pixel_radius = int((world_radius / world_z) * focal_length)
    #     return max(pixel_radius, 5)  # 最小半径为5像素

    def draw_target_overlay(self, image, target):
        """在图像上绘制单个目标"""
        try:
            # 转换世界坐标到图像坐标
            # img_x, img_y = self.world_to_image(target['x'], target['y'], target['z'])
            
            # if img_x is None or img_y is None:
            #     return  # 目标不在图像范围内
            
            # 计算像素半径
            # pixel_radius = self.calculate_pixel_radius(target['radius'], target['z'])
            
            # 确保坐标和半径都是整数类型，并进行边界检查
            center_x = int(float(target['x']))
            center_y = int(float(target['y']))
            radius = max(1, int(float(target['radius'])))  # 确保半径至少为1
            
            # 检查坐标是否在图像范围内
            if (center_x < 0 or center_x >= image.shape[1] or 
                center_y < 0 or center_y >= image.shape[0]):
                self.get_logger().debug(f"目标 {target['id']} 超出图像范围: ({center_x}, {center_y})")
                return
            
            # 绘制圆形
            color = (target['color']['b'], target['color']['g'], target['color']['r'])  # BGR格式
            thickness = 2
            
            # 绘制外圆
            cv2.circle(image, (center_x, center_y), radius, color, thickness)

            # 绘制中心点
            cv2.circle(image, (center_x, center_y), 3, color, -1)
            
            # 绘制目标信息文本
            text_lines = [
                f"ID: {target['id']}",
                f"Cat: {target['category']}",
                f"Pos: ({target['x']:.1f}, {target['y']:.1f}, {target['z']:.1f})",
                f"R: {target['radius']:.1f}px"
            ]
            
            # 计算文本位置（在圆形上方）
            text_x = center_x - 50
            text_y = center_y - radius - 10
            
            # 绘制文本背景
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.4
            text_thickness = 1
            
            # 确保文本位置在图像范围内
            if text_y < 0:
                text_y = center_y + radius + 20  # 如果上方没有空间，在下方显示
            
            for i, text in enumerate(text_lines):
                y_offset = text_y + i * 15
                
                # 确保文本不会超出图像边界
                if y_offset < 0 or y_offset >= image.shape[0]:
                    continue
                if text_x < 0:
                    text_x = 10
                if text_x >= image.shape[1] - 100:
                    text_x = image.shape[1] - 110
                
                # 获取文本尺寸
                (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, text_thickness)
                
                # 绘制文本背景
                bg_x1 = max(0, text_x - 2)
                bg_y1 = max(0, y_offset - text_height - 2)
                bg_x2 = min(image.shape[1] - 1, text_x + text_width + 2)
                bg_y2 = min(image.shape[0] - 1, y_offset + 2)
                
                cv2.rectangle(image, 
                             (bg_x1, bg_y1),
                             (bg_x2, bg_y2),
                             (0, 0, 0), -1)
                
                # 绘制文本
                cv2.putText(image, text, (text_x, y_offset), 
                           font, font_scale, (255, 255, 255), text_thickness)
                       
        except Exception as e:
            self.get_logger().error(f'绘制目标 {target.get("id", "unknown")} 时出错: {str(e)}')
            self.get_logger().debug(f'目标数据: {target}')

    def image_callback(self, msg: Image):
        """处理接收到的图像消息"""
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 更新图像尺寸
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # 复制图像用于叠加
            overlay_image = cv_image.copy()
            
            # 在图像上绘制所有目标
            for target in self.current_targets:
                self.draw_target_overlay(overlay_image, target)
            
            # 绘制状态信息
            status_text = f"Targets: {len(self.current_targets)}"
            cv2.putText(overlay_image, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 绘制时间戳
            import time
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            cv2.putText(overlay_image, f"Time: {timestamp}", (10, overlay_image.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 绘制图像尺寸信息
            size_text = f"Size: {self.image_width}x{self.image_height}"
            cv2.putText(overlay_image, size_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 绘制使用说明
            help_text = "Press 'q' to quit, 's' to save screenshot"
            cv2.putText(overlay_image, help_text, (10, overlay_image.shape[0] - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            
            # 显示处理后的图像
            cv2.imshow(self.window_name, overlay_image)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("用户按下 'q' 键，退出程序")
                rclpy.shutdown()
            elif key == ord('s'):
                # 保存截图
                import os
                screenshot_dir = "/tmp/target_visualization_screenshots"
                os.makedirs(screenshot_dir, exist_ok=True)
                filename = f"{screenshot_dir}/screenshot_{int(time.time())}.jpg"
                cv2.imwrite(filename, overlay_image)
                self.get_logger().info(f"截图已保存到: {filename}")
            
            # 转换回ROS消息并发布
            # overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, "bgr8")
            # overlay_msg.header = msg.header  # 保持原始时间戳
            # self.overlay_publisher.publish(overlay_msg)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')

    def cleanup(self):
        """清理资源"""
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        visualizer = TargetOverlayVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        if 'visualizer' in locals():
            visualizer.cleanup()
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
