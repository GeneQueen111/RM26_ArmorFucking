#!/bin/bash

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

# 加载 OpenVINO 环境 - 首先尝试从工作区父目录定位到 openvino* 安装目录
WORKSPACE_PARENT="$(dirname "$WORKSPACE_DIR")"

find_openvino_root() {
    local base_dir="$1"
    local candidate
    for candidate in "$base_dir"/openvino*; do
        [ -e "$candidate" ] || continue
        if [ -f "$candidate/setupvars.sh" ] || [ -f "$candidate/bin/setupvars.sh" ]; then
            printf '%s' "$candidate"
            return 0
        fi
    done
    return 1
}

if [ -z "$OPENVINO_DIR" ]; then
    OPENVINO_DIR="$(find_openvino_root "$WORKSPACE_PARENT")"
fi

setupvars_file=""
if [ -n "$OPENVINO_DIR" ]; then
    if [ -f "$OPENVINO_DIR/setupvars.sh" ]; then
        setupvars_file="$OPENVINO_DIR/setupvars.sh"
    elif [ -f "$OPENVINO_DIR/bin/setupvars.sh" ]; then
        setupvars_file="$OPENVINO_DIR/bin/setupvars.sh"
    fi
fi

if [ -n "$setupvars_file" ]; then
    echo "正在加载 OpenVINO 环境 ($setupvars_file)"
    # shellcheck disable=SC1090
    source "$setupvars_file"
else
    echo "警告: 未找到 OpenVINO setupvars.sh，推理可能失败"
fi

# 先执行增量编译
echo "正在执行增量编译..."
bash "$WORKSPACE_DIR/incremental_build.sh"

# 检查编译是否成功
if [ $? -ne 0 ]; then
    echo "编译失败，停止启动节点"
    exit 1
fi

# 检查 setup.bash 是否存在
SETUP_FILE="$WORKSPACE_DIR/install/setup.bash"
if [ ! -f "$SETUP_FILE" ]; then
    echo "错误: 找不到 $SETUP_FILE"
    echo "请确保已经编译了工作空间"
    exit 1
fi

echo "正在启动 ROS2 节点..."
echo "工作空间目录: $WORKSPACE_DIR"

# Source 环境
echo "正在加载 ROS2 环境..."
source "$SETUP_FILE"

echo "环境加载完成，开始启动节点..."

# 使用 launch 文件启动相机节点（会自动加载 YAML 配置）
ros2 launch hik_camera hik_camera.launch.py &
echo "启动 hik_camera_node (后台运行，使用 launch 文件)..."
HIK_PID=$!

sleep 1

echo "启动 recive_pkg (后台运行)..."
ros2 run recive_pkg recive_pkg &
RECIVE_PID=$!

echo "所有节点已启动完成!"
echo "进程 ID:"
echo "  hik_camera_node: $HIK_PID"
echo "  recive_pkg: $RECIVE_PID"
echo ""
echo "要停止节点，请按 Ctrl+C 或运行:"
echo "  kill $HIK_PID $RECIVE_PID"

# 等待用户中断
trap "echo '正在停止节点...'; kill $HIK_PID $RECIVE_PID 2>/dev/null; echo '节点已停止'; exit 0" INT

# 保持脚本运行
wait
