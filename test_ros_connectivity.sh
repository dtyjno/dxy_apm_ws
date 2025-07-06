#!/bin/bash

# ROS 2 网络连通性测试脚本

echo "=== ROS 2 网络连通性测试 ==="

# 检查ROS 2环境变量
echo "1. 检查ROS 2环境变量:"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

# 检查网络接口
echo "2. 检查网络接口:"
ip addr show | grep -E "(inet|UP)" | head -10
echo ""

# 检查多播支持
echo "3. 检查多播支持:"
if [ -e /proc/net/igmp ]; then
    echo "   多播支持: 可用"
    cat /proc/net/igmp | head -5
else
    echo "   多播支持: 不可用"
fi
echo ""

# 列出当前ROS 2节点
echo "4. 列出当前ROS 2节点:"
timeout 5s ros2 node list 2>/dev/null || echo "   无法获取节点列表或超时"
echo ""

# 列出当前ROS 2话题
echo "5. 列出当前ROS 2话题:"
timeout 5s ros2 topic list 2>/dev/null || echo "   无法获取话题列表或超时"
echo ""

# 测试发布/订阅
echo "6. 测试ROS 2通信:"
echo "   启动测试发布者（后台运行10秒）..."
timeout 10s ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello from container'" --rate 1 &
PUB_PID=$!

sleep 2

echo "   检查是否能接收到消息..."
timeout 5s ros2 topic echo /test_topic --once 2>/dev/null && echo "   ✓ 通信测试成功" || echo "   ✗ 通信测试失败"

# 清理后台进程
kill $PUB_PID 2>/dev/null

echo ""
echo "=== 测试完成 ==="

# 提供故障排除建议
echo ""
echo "故障排除建议:"
echo "1. 确保主机和容器使用相同的ROS_DOMAIN_ID"
echo "2. 确保ROS_LOCALHOST_ONLY=0以允许跨容器通信"
echo "3. 检查防火墙设置是否阻止了多播流量"
echo "4. 确保使用--network host模式运行容器"
echo "5. 在主机上测试: ros2 topic list"
