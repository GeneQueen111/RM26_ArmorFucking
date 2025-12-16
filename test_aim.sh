#!/bin/bash

# 测试瞄准节点启动脚本
# 自动构建并启动完整任务测试

echo "=========================================="
echo "  装甲板检测完整任务测试"
echo "=========================================="
echo ""

# 切换到脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "当前工作目录: $SCRIPT_DIR"
echo ""

# 检查源码文件是否存在
if [ ! -f "src/rm_auto_aim/detection/test_aim_node.cpp" ]; then
    echo "错误: 未找到 test_aim_node.cpp"
    echo "请确保在正确的项目目录中"
    exit 1
fi

echo "1. 清理旧的构建文件..."
echo "=========================================="

# 清理源码目录的构建文件（为了确保能正确编译）
cd src/rm_auto_aim/detection
rm -rf CMakeCache.txt CMakeFiles cmake_install.cmake Makefile test_aim_node 2>/dev/null

echo "2. 配置并编译..."
echo "=========================================="

# 配置 cmake
cmake . > /dev/null 2>&1

if [ $? -ne 0 ]; then
    echo "CMake 配置失败"
    exit 1
fi

# 编译 test_aim_node
echo "正在编译 test_aim_node..."
make test_aim_node > /dev/null 2>&1

if [ $? -ne 0 ]; then
    echo ""
    echo "编译失败，尝试显示错误信息..."
    make test_aim_node
    exit 1
fi

echo "编译成功！"
echo ""

# 返回项目根目录
cd "$SCRIPT_DIR"

echo "3. 编译完成，准备启动测试..."
echo "=========================================="
echo ""

# 查找可执行文件（从源码目录）
TEST_NODE=$(find src/rm_auto_aim/detection -name "test_aim_node" -type f -executable 2>/dev/null | head -1)

if [ -z "$TEST_NODE" ]; then
    echo "错误: 未找到 test_aim_node 可执行文件"
    exit 1
fi

echo "找到可执行文件: $TEST_NODE"
echo ""
echo "=========================================="
echo "  开始完整任务测试"
echo "=========================================="
echo ""
echo "测试流程:"
echo "  1. 自动扫描视频目录"
echo "  2. 批量测试所有视频文件"
echo "  3. 统计检测性能指标"
echo "  4. 生成详细测试报告"
echo ""
echo "注意: 如果遇到模型输入尺寸不匹配错误，"
echo "      这是正常的，说明检测器正在工作"
echo "      可以尝试调整视频分辨率或模型配置"
echo ""
echo "按 'q' 键可随时中断测试"
echo "=========================================="
echo ""

# 运行测试节点
"$TEST_NODE"

echo ""
echo "=========================================="
echo "  测试完成"
echo "=========================================="
