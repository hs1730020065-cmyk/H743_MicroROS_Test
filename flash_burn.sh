#!/bin/bash
# ============================================================
#  STM32H743 编译 + 烧录脚本 (正点原子 DAP / CMSIS-DAP)
#  用法: 在 Git Bash 中运行 ./flash_burn.sh
# ============================================================
set -e

# ---- 路径配置 ----
STM32CUBE_IDE_DIR="D:/stm32cubeide/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins"

# arm-none-eabi-gcc 工具链
GCC_DIR="${STM32CUBE_IDE_DIR}/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin"

# OpenOCD 可执行文件
OPENOCD="${STM32CUBE_IDE_DIR}/com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.400.202601091506/tools/bin/openocd.exe"

# OpenOCD 脚本目录 (接口 + 目标配置文件)
SCRIPTS="${STM32CUBE_IDE_DIR}/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"

# 工程目录
PROJECT_DIR="D:/STM32Project/H743_MicroROS_Test"
BUILD_DIR="${PROJECT_DIR}/Debug"
HEX_FILE="${BUILD_DIR}/H743_MicroROS_Test.hex"

# ============================================================
#  1. 编译
# ============================================================
echo "===== 编译工程 ====="
cd "${BUILD_DIR}"
export PATH="${GCC_DIR}:${PATH}"
make -j8 all
echo "编译完成."

# ============================================================
#  2. 烧录 (CMSIS-DAP)
# ============================================================
echo ""
echo "===== 烧录固件 (CMSIS-DAP) ====="
"${OPENOCD}" \
    -s "${SCRIPTS}" \
    -f interface/cmsis-dap.cfg \
    -c "transport select swd" \
    -c "set AP_NUM 0" \
    -c "set CORE_RESET 0" \
    -f target/stm32h7x.cfg \
    -c "init" \
    -c "halt" \
    -c "program ${HEX_FILE} verify" \
    -c "reset run" \
    -c "exit"

echo ""
echo "===== 完成! ====="
echo "如果板子没有自动复位，请手动按 RESET 按钮或重新上电。"
