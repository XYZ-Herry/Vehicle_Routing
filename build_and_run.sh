#!/bin/bash

# 设置输入文件路径
INPUT_FILE="../test/output_data_weighted.txt"
OUTPUT_FILE="../test/result.txt"

# 创建并进入构建目录
mkdir -p build
cd build

# 运行 CMake 生成构建文件
cmake ..

# 编译项目
make

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "构建成功，正在运行程序..."
    if [ -f "./delivery_system" ]; then
        # 运行程序并将输出重定向到文件
        ./delivery_system "$INPUT_FILE" > "$OUTPUT_FILE"
        echo "程序输出已保存到 $OUTPUT_FILE"
    else
        echo "错误: 可执行文件未找到"
        exit 1
    fi
else
    echo "构建失败"
    exit 1
fi 