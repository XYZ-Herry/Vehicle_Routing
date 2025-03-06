# 车辆路径规划系统

这是一个用于解决车辆和无人机混合配送路径规划问题的系统。系统使用三阶段法求解：任务分配、遗传算法优化和路径优化。

## 项目结构 
├── include/ # 头文件目录
├── src/ # 源代码目录
├── test/ # 测试数据和结果目录
├── graph/ # 可视化工具目录
├── CMakeLists.txt # CMake 构建文件
└── README.md # 项目说明文档
## 编译步骤

1. 创建并进入构建目录：
bash
mkdir build
cd build
bash
cmake ..

3. 编译项目：
```bash
make
```

## 运行程序

1. 运行主程序（在 build 目录下）：
```bash
./delivery_system ../test/output_data_weighted.txt > ../test/result.txt
```

或者同时查看输出并保存到文件：
```bash
./delivery_system ../test/output_data_weighted.txt | tee ../test/result.txt
```

## 可视化配送路径

1. 确保已安装 Python3 和 matplotlib：
```bash
# Ubuntu/Debian
sudo apt-get install python3 python3-matplotlib

# CentOS/Fedora
sudo yum install python3 python3-matplotlib

# 或使用 pip 安装
pip3 install matplotlib
```

2. 运行可视化脚本：
```bash
./graph/visualize.sh ../test/output_data_weighted.txt ../test/result.txt route_map.png
```

## 输入文件格式

输入文件包含以下信息：
- 初始需求点数量和额外需求点数量
- 车辆配送中心数量和无人机配送中心数量
- 车辆和无人机的参数（速度、成本、载重等）
- 路网信息（边的数量和连接关系）
- 任务点坐标（经纬度）
- 配送中心信息（位置和车辆/无人机数量）

## 输出说明

程序输出包括：
1. 坐标信息
   - 初始需求点坐标
   - 车辆配送中心坐标
   - 无人机配送中心坐标
   - 额外需求点坐标

2. 配送方案
   - 每个配送工具的任务序列
   - 每个任务的完成时间

3. 可视化结果
   - 蓝点：初始需求点
   - 绿点：额外需求点
   - 红方块：车辆配送中心
   - 紫方块：无人机配送中心
   - 彩色线段：配送路径

## 注意事项

1. 确保输入文件格式正确
2. 运行可视化工具前需要安装必要的 Python 包
3. 所有坐标均使用经纬度表示，程序内部会自动转换为公里单位
4. 输出的时间单位为小时，距离单位为公里

## 示例

```bash
# 编译项目
mkdir build && cd build
cmake ..
make

# 运行程序
./delivery_system ../test/output_data_weighted.txt > ../test/result.txt

# 可视化结果
./graph/visualize.sh ../test/output_data_weighted.txt ../test/result.txt route_map.png
```