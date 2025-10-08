#!/bin/bash

# 固件路径
FILE="build/Infantry.elf"

# 检查文件是否存在
if [ ! -f "$FILE" ]; then
    echo "[ERROR] 文件未找到: $FILE"
    exit 1
fi

# 执行 OpenOCD 烧录
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program $FILE verify reset exit"
# openocd -f interface/Jlink.cfg -f target/stm32f4x.cfg -c "program $FILE verify reset exit"

# 检查返回状态
if [ $? -ne 0 ]; then
    echo "[ERROR] 烧录失败！"
    exit 1
fi

echo "[OK] 烧录成功！"
