#!/bin/bash

# 设置输入文件路径
INPUT_FILE="../test/1.txt"
OUTPUT_FILE="../test/result.txt"

# 创建并进入构建目录
mkdir -p build
cd build

# 使用Debug模式运行CMake（保留调试信息但不影响正常运行）
cmake -DCMAKE_BUILD_TYPE=Debug ..

# 编译项目
make

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "构建成功，正在运行程序..."
    if [ -f "./delivery_system" ]; then
        # 先尝试正常运行程序
        ./delivery_system "$INPUT_FILE" > "$OUTPUT_FILE"
        
        # 检查程序退出状态
        if [ $? -ne 0 ]; then
            echo "程序运行出错，启动GDB调试..."
            gdb -ex "catch throw" -ex "run" --args ./delivery_system "$INPUT_FILE"
        else
            echo "程序成功运行，输出已保存到 $OUTPUT_FILE"
        fi
    else
        echo "错误: 可执行文件未找到"
        exit 1
    fi
else
    echo "构建失败"
    exit 1
fi 