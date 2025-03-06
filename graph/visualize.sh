#!/bin/bash

# 检查Python是否安装
if ! command -v python3 &> /dev/null; then
    echo "Error: Python3 is required but not installed."
    echo "Please install Python3 using your package manager."
    exit 1
fi

# 检查matplotlib是否安装
python3 -c "import matplotlib" &> /dev/null
if [ $? -ne 0 ]; then
    echo "Installing matplotlib..."
    pip3 install matplotlib
fi

# 检查参数
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <data_file> <result_file> [output_image]"
    exit 1
fi

DATA_FILE=$1
RESULT_FILE=$2
OUTPUT_IMAGE=${3:-"route_visualization.png"}

# 运行可视化脚本
python3 $(dirname "$0")/visualize.py --data "$DATA_FILE" --result "$RESULT_FILE" --output "$OUTPUT_IMAGE" 