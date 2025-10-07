#!/bin/bash

# 一旦出现错误，立即退出脚本
set -e

# 进入当前脚本所在目录
cd "$(dirname "$0")"

# 删除旧的 build 目录（如果存在）
if [ -d "build" ]; then
    echo "Deleting old build directory..."
    rm -rf build
fi

# 创建新的 build 目录
mkdir build
cd build

# 运行 CMake 配置（使用 MinGW Makefiles 和指定工具链文件）
echo "Configuring CMake..."
cmake -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake .. || {
    echo "❌ CMake configuration failed!"
    exit 1
}

# 编译工程
echo "Starting build..."
cmake --build . -- -j 4 || {
    echo "❌ Build failed!"
    exit 1
}

# 编译完成后，提示信息
echo "✅ Build completed successfully!"
