# 车辆路径规划系统

![版本](https://img.shields.io/badge/版本-1.0-blue)
![构建状态](https://img.shields.io/badge/构建-通过-brightgreen)

## 📖 项目简介

这是一个用于解决车辆和无人机混合配送路径规划问题的系统。系统使用以下方法求解：
- **任务分配**：合理分配各个配送点
- **静态阶段遗传算法优化**：静态阶段使用遗传算法优化每个车辆/无人机分配的任务点
- **路径优化**：进一步优化各个车辆/无人机的路径
- **动态阶段遗传算法优化**：动态阶段将静态阶段由于道路情况而超过时间T的任务以及新加入的任务重新分配给无人机/车辆
- **路径优化**：统筹规划车机协同优化所有路径

## 🗂️ 项目结构 

```
├── include/         # 头文件目录
├── src/             # 源代码目录
├── test/            # 测试数据和结果目录
├── graph/           # 可视化工具目录
├── docs/            # 文档目录
├── CMakeLists.txt   # CMake 构建文件
└── README.md        # 项目说明文档
```

## 🚀 编译运行步骤

### 方法一：手动编译

1. 创建并进入构建目录：
   ```bash
   mkdir build
   cd build
   ```

2. 运行 CMake：
   ```bash
   cmake ..
   ```

3. 编译项目：
   ```bash
   make
   ```

4. 运行程序（在build目录下）：
   ```bash
   ./delivery_system ../test/output_data_weighted.txt > ../test/result.txt
   ```

### 方法二：使用脚本编译运行

直接在项目主目录下执行：
```bash
chmod +x build_and_run.sh
./build_and_run.sh
```
结果将生成在主目录下的result.txt文件里面

## 📊 输入输出说明

### 输入文件格式
详细的输入文件格式说明请参见 `docs` 目录下的 README.md 文件。

### 输出结果
系统将生成优化后的配送路径，包括车辆和无人机的协同配送计划。

## 📝 使用示例

1. 准备输入数据文件
2. 按照上述步骤编译运行系统
3. 查看 `test/result.txt` 中的优化结果

## 👥 贡献者

- XYZ-Herry

## 📄 许可证

MIT License