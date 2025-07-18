#!/usr/bin/env python3
"""
简化的图像圆圈叠加工具
直接从文件读取图像，根据指定的目标数据叠加圆圈标记
"""

import cv2
import numpy as np
import json
import argparse
import os
import sys

class ImageCircleOverlay:
    def __init__(self):
        self.colors = {
            'circle': (0, 255, 0),      # 绿色 - 圆形目标
            'H_type': (0, 0, 255),      # 红色 - H型目标
            'default': (255, 255, 0)    # 青色 - 默认
        }
    
    def load_targets_from_json(self, json_file):
        """从JSON文件加载目标数据"""
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            return data.get('targets', [])
        except Exception as e:
            print(f"加载JSON文件失败: {e}")
            return []
    
    def create_sample_targets(self):
        """创建示例目标数据"""
        return [
            {
                'id': 1,
                'category': 'circle',
                'center_x': 300,
                'center_y': 200,
                'radius': 50,
                'confidence': 0.95
            },
            {
                'id': 2,
                'category': 'H_type',
                'center_x': 600,
                'center_y': 400,
                'radius': 30,
                'confidence': 0.87
            },
            {
                'id': 3,
                'category': 'circle',
                'center_x': 800,
                'center_y': 300,
                'radius': 40,
                'confidence': 0.92
            }
        ]
    
    def draw_circle_target(self, image, target):
        """在图像上绘制圆形目标"""
        center_x = int(target['center_x'])
        center_y = int(target['center_y'])
        radius = int(target['radius'])
        category = target.get('category', 'default')
        target_id = target.get('id', 0)
        confidence = target.get('confidence', 1.0)
        
        # 获取颜色
        color = self.colors.get(category, self.colors['default'])
        
        # 绘制外圆
        cv2.circle(image, (center_x, center_y), radius, color, 2)
        
        # 绘制中心点
        cv2.circle(image, (center_x, center_y), 3, color, -1)
        
        # 绘制十字标记
        cross_size = 10
        cv2.line(image, 
                (center_x - cross_size, center_y), 
                (center_x + cross_size, center_y), 
                color, 2)
        cv2.line(image, 
                (center_x, center_y - cross_size), 
                (center_x, center_y + cross_size), 
                color, 2)
        
        # 绘制标签文本
        label_lines = [
            f"ID: {target_id}",
            f"Cat: {category}",
            f"R: {radius}px",
            f"Conf: {confidence:.2f}"
        ]
        
        # 计算文本位置
        text_x = center_x + radius + 10
        text_y = center_y - len(label_lines) * 8
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        font_thickness = 1
        
        for i, label in enumerate(label_lines):
            y_pos = text_y + i * 16
            
            # 文本背景
            (text_width, text_height), _ = cv2.getTextSize(label, font, font_scale, font_thickness)
            cv2.rectangle(image, 
                         (text_x - 2, y_pos - text_height - 2),
                         (text_x + text_width + 2, y_pos + 2),
                         (0, 0, 0), -1)
            
            # 绘制文本
            cv2.putText(image, label, (text_x, y_pos), 
                       font, font_scale, (255, 255, 255), font_thickness)
    
    def overlay_targets_on_image(self, image_path, targets, output_path=None):
        """在图像上叠加所有目标"""
        # 读取图像
        image = cv2.imread(image_path)
        if image is None:
            print(f"无法读取图像: {image_path}")
            return None
        
        # 复制图像
        result_image = image.copy()
        
        # 绘制所有目标
        for target in targets:
            self.draw_circle_target(result_image, target)
        
        # 绘制统计信息
        stats_text = f"Targets: {len(targets)}"
        cv2.putText(result_image, stats_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 绘制图例
        self.draw_legend(result_image)
        
        # 保存结果
        if output_path:
            cv2.imwrite(output_path, result_image)
            print(f"结果已保存到: {output_path}")
        
        return result_image
    
    def draw_legend(self, image):
        """绘制图例"""
        legend_x = image.shape[1] - 200
        legend_y = 50
        
        legend_items = [
            ('circle', 'Circle Target'),
            ('H_type', 'H-type Target')
        ]
        
        for i, (category, label) in enumerate(legend_items):
            y_pos = legend_y + i * 30
            color = self.colors[category]
            
            # 绘制图例圆圈
            cv2.circle(image, (legend_x, y_pos), 8, color, 2)
            
            # 绘制图例文本
            cv2.putText(image, label, (legend_x + 20, y_pos + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

def main(args=None):
    parser = argparse.ArgumentParser(description='图像圆圈叠加工具')
    parser.add_argument('--image', type=str, help='输入图像路径')
    parser.add_argument('--targets', type=str, help='目标数据JSON文件路径')
    parser.add_argument('--output', type=str, help='输出图像路径')
    parser.add_argument('--demo', action='store_true', help='使用示例数据运行演示')
    
    args = parser.parse_args()
    
    overlay_tool = ImageCircleOverlay()
    
    if args.demo:
        # 演示模式
        targets = overlay_tool.create_sample_targets()
        print("演示模式 - 使用示例目标数据")
        print(f"生成了 {len(targets)} 个示例目标")
        
        if args.image:
            result = overlay_tool.overlay_targets_on_image(args.image, targets, args.output)
            if result is not None:
                cv2.imshow('Target Overlay Demo', result)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
        else:
            print("演示模式需要指定图像路径 (--image)")
    
    elif args.image:
        # 单图像处理模式
        if args.targets:
            targets = overlay_tool.load_targets_from_json(args.targets)
        else:
            targets = overlay_tool.create_sample_targets()
            print("未指定目标数据，使用示例数据")
        
        result = overlay_tool.overlay_targets_on_image(args.image, targets, args.output)
        
        if result is not None:
            cv2.imshow('Target Overlay', result)
            print("按任意键关闭窗口...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    
    else:
        print("请指定输入参数。使用 --help 查看帮助信息")

if __name__ == '__main__':
    main()
