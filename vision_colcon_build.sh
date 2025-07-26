#!/bin/bash
                # ('model_path1', './src/ros2_yolov8/ros2_v8/best_circle.pt'),
                # ('model_path2', './src/ros2_yolov8/ros2_v8/best_H.pt'),
conda deactivate 2>/dev/null
source /opt/ros/jazzy/setup.bash;
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ros_yolo

# Verify we're using system Python
echo "Using Python: $(which python)"
echo "Python version: $(python --version)"
echo "PYTHONPATH: $PYTHONPATH"

# Clean build and rebuild with system environment
rm -rf build/ros_yolo install/ros_yolo
python -m colcon build --packages-select ros_yolo --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release;


# gnome-terminal -t "ros2" -x bash -c "#cd ~/ws_sensor_combined/;
#cd ~/ros2_ws;
# cd ~/ardupilot_ws;
# source /opt/ros/humble/setup.bash;
#colcon build;
# colcon build --packages-select px4_ros_com; 
#cd ~/ros2_ws;
#colcon build --packages-up-to ardupilot_gz_bringup;
# exec bash;"

# cd ~/ardupilot_ws;
# source /opt/ros/humble/setup.bash;
# MAKEFLAGS="-j1 " colcon build --executor sequential --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON;

# self._load_camera_calibration('./src/ros2_yolov8/ros2_v8/rgb_camera_calib_1.npz')

# def _load_camera_calibration(self, path):
#     try:
#         calib_data = np.load(path)
#         self.camera_matrix = calib_data['camera_matrix']
#         self.dist_coeffs = calib_data['dist_coeffs']
        
#         # 提取相机内参
#         fx = float(self.camera_matrix[0, 0])
#         fy = float(self.camera_matrix[1, 1])
#         cx = float(self.camera_matrix[0, 2])
#         cy = float(self.camera_matrix[1, 2])
        
#         # 展平畸变系数数组并提取畸变系数
#         dist_flat = self.dist_coeffs.flatten()
#         k1 = float(dist_flat[0]) if len(dist_flat) > 0 else 0.0
#         k2 = float(dist_flat[1]) if len(dist_flat) > 1 else 0.0
#         p1 = float(dist_flat[2]) if len(dist_flat) > 2 else 0.0
#         p2 = float(dist_flat[3]) if len(dist_flat) > 3 else 0.0
#         k3 = float(dist_flat[4]) if len(dist_flat) > 4 else 0.0
        
#         self.get_logger().info("成功加载相机标定参数")
#         self.get_logger().info("=== 相机标定参数详情 ===")
#         self.get_logger().info(f"fx: {fx:.1f}")
#         self.get_logger().info(f"fy: {fy:.1f}")
#         self.get_logger().info(f"cx: {cx:.1f}")
#         self.get_logger().info(f"cy: {cy:.1f}")
#         self.get_logger().info(f"k1: {k1:.3f}")
#         self.get_logger().info(f"k2: {k2:.3f}")
#         self.get_logger().info(f"p1: {p1:.3f}")
#         self.get_logger().info(f"p2: {p2:.3f}")
#         self.get_logger().info(f"k3: {k3:.3f}")
#         self.get_logger().info("========================")
        
#     except Exception as e:
#         self.get_logger().error(f"加载标定参数失败: {str(e)}")
#         self.camera_matrix = None
#         self.dist_coeffs = None


#     self.declare_parameters(
#         namespace='',
#         parameters=[
#             ('camera_id', ''),
#             #('image_topic', 'raw_images'),
#             ('image_topic', 'image_topic'),  # 图像订阅话题
#             ('model_path1', './src/ros2_yolov8/ros2_v8/best_circle.pt'),
#             ('model_path2', './src/ros2_yolov8/ros2_v8/best_H.pt'),
#             ('conf_threshold', 0.6),
#             ('device', 'cuda:0'),
#             ('frame_size', [1920, 1080]),
#             ('publish_raw', True),
#         ]
#     )
