from setuptools import find_packages, setup

package_name = 'target_visualization'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/target_visualization.launch.py']),
        ('share/' + package_name + '/config', ['config/sample_targets.json']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='linhao',
    maintainer_email='23050820@hdu.edu.cn',
    description='ROS2 package for target visualization overlay on images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_overlay_visualizer = target_visualization.target_overlay_visualizer:main',
            'test_publisher = target_visualization.test_publisher:main',
            'image_circle_overlay = target_visualization.image_circle_overlay:main',
        ],
    },
)
