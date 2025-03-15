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
```bash
mkdir build
cd build
```

2. 运行Cmake：
```bash
cmake ..
```

3. 编译项目：
```bash
make
```

4. 运行程序
在build目录下
```bash
./delivery_system ../test/output_data_weighted.txt > ../test/result.txt
```

或者直接在项目主目录下
```bash
chmod +x build_and_run.sh
./build_and_run.sh
```

输入文件格式 见docs目录下的README.md文件