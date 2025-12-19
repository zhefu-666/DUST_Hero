#!/bin/bash
# 启动脚本 - 自动设置X11显示环境

# 设置X11环境变量
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# 检查X11连接
if ! xdpyinfo > /dev/null 2>&1; then
    echo "⚠ 无法连接到X11显示器"
    echo "尝试其他显示器..."
    export DISPLAY=:0
    export XAUTHORITY=/run/user/1000/gdm/Xauthority
    if ! xdpyinfo > /dev/null 2>&1; then
        echo "❌ 没有可用的X11显示器"
        echo "请确保："
        echo "  1. 已登录图形界面"
        echo "  2. 或使用 ssh -X 连接"
        exit 1
    fi
fi

echo "✓ X11 环境配置成功"
echo "  DISPLAY=$DISPLAY"
echo "  XAUTHORITY=$XAUTHORITY"
echo ""
echo "启动 TJURM-2024..."

# 切换到项目目录并运行
cd /home/hero/DUST_Hero
./build/TJURM-2024 "$@"
