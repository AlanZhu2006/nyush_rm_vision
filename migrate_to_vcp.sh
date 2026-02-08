#!/bin/bash
# 自动将所有使用CBoard的程序迁移到Gimbal（VCP）
# 作者：Claude Code
# 日期：2026-02-08

echo "================================================"
echo "  迁移脚本：CBoard (CAN) → Gimbal (VCP)"
echo "================================================"
echo ""
echo "此脚本将："
echo "  1. 替换所有源文件中的 io::CBoard → io::Gimbal"
echo "  2. 替换头文件 io/cboard.hpp → io/gimbal/gimbal.hpp"
echo "  3. 更新CMakeLists.txt链接库"
echo ""
read -p "确认执行？(y/n): " confirm

if [ "$confirm" != "y" ]; then
    echo "取消操作"
    exit 0
fi

echo ""
echo "开始迁移..."
echo ""

# 备份列表
BACKUP_DIR="backup_cboard_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

# 需要修改的文件列表（主程序和测试）
FILES_TO_MODIFY=(
    "src/standard.cpp"
    "src/mt_standard.cpp"
    "src/standard_mpc.cpp"
    "src/auto_aim_debug_mpc.cpp"
    "src/mt_auto_aim_debug.cpp"
    "src/auto_buff_debug.cpp"
    "src/auto_buff_debug_mpc.cpp"
    "src/uav.cpp"
    "src/uav_debug.cpp"
    "calibration/capture.cpp"
    "tests/handeye_test.cpp"
    "tests/auto_aim_test.cpp"
    "tests/auto_buff_test.cpp"
    "tests/minimum_vision_system.cpp"
)

# 步骤1：备份并替换源文件
echo "[1/3] 替换源文件中的 CBoard → Gimbal..."
for file in "${FILES_TO_MODIFY[@]}"; do
    if [ -f "$file" ]; then
        echo "  处理: $file"

        # 备份
        cp "$file" "$BACKUP_DIR/"

        # 替换 include
        sed -i 's/#include "io\/cboard\.hpp"/#include "io\/gimbal\/gimbal.hpp"/g' "$file"

        # 替换类名（保留变量名cboard，只改类型）
        sed -i 's/io::CBoard cboard/io::Gimbal cboard/g' "$file"

        echo "    ✓ 完成"
    else
        echo "    ⚠ 跳过（文件不存在）: $file"
    fi
done

echo ""
echo "[2/3] 更新CMakeLists.txt..."

# 备份CMakeLists.txt
cp CMakeLists.txt "$BACKUP_DIR/"

# 注意：Gimbal已经在io库中，不需要修改链接库

echo "    ✓ CMakeLists.txt已检查（无需修改，io库同时包含CBoard和Gimbal）"

echo ""
echo "[3/3] 更新配置文件..."

# 配置文件列表
CONFIGS=(
    "configs/standard3.yaml"
    "configs/standard4.yaml"
    "configs/sentry.yaml"
    "configs/uav.yaml"
    "configs/example.yaml"
)

for config in "${CONFIGS[@]}"; do
    if [ -f "$config" ]; then
        echo "  处理: $config"

        # 备份
        cp "$config" "$BACKUP_DIR/"

        # 检查是否已有com_port参数
        if grep -q "^com_port:" "$config"; then
            echo "    ℹ 已有com_port参数，跳过"
        else
            # 在cboard参数部分后添加gimbal参数
            # 使用awk在文件末尾添加
            cat >> "$config" << 'EOF'

#####-----gimbal参数（VCP串口）-----#####
com_port: "/dev/gimbal"  # USB虚拟串口设备
EOF
            echo "    ✓ 添加com_port参数"
        fi
    else
        echo "    ⚠ 跳过（文件不存在）: $config"
    fi
done

echo ""
echo "================================================"
echo "  迁移完成！"
echo "================================================"
echo ""
echo "备份文件位置: $BACKUP_DIR/"
echo ""
echo "下一步："
echo "  1. 检查修改的文件（使用 git diff）"
echo "  2. 编译测试："
echo "     cd build && cmake .. && make -j12"
echo "  3. 运行测试程序验证"
echo ""
echo "如需回滚："
echo "  cp -r $BACKUP_DIR/* ."
echo ""
