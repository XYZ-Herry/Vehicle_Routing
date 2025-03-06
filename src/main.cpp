#include <iostream>
#include <vector>
#include <limits>
#include <string>
#include "common.h"
#include "solver.h"

using std::vector;
using std::pair;
using std::cout;
using std::endl;
using std::string;

int main(int argc, char* argv[])
{
    // 初始化随机数生成器
    srand(time(0));
    
    // 检查命令行参数
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " <input_file>" << endl;
        cout << "Example: " << argv[0] << " ../test/output_data_weighted.txt" << endl;
        return 1;
    }
    
    // 获取输入文件路径
    string filename = argv[1];
    
    // 初始化并加载配送问题数据
    DeliveryProblem problem;
    if (!loadProblemData(filename, problem)) {
        cout << "加载数据失败，程序退出。" << endl;
        return 1;
    }
    
    // 使用三阶段法求解静态配送问题
    // 1. 任务分配到最近配送中心
    // 2. 使用遗传算法为车辆分配任务
    // 3. 优化每个车辆的配送路径
    auto allPaths = solveStaticProblem(problem);
    
    // 输出求解结果
    for (size_t i = 0; i < allPaths.size(); ++i) {
        if (allPaths[i].first.empty()) continue;
        
        cout << "配送工具 " << i << " (";
        if (problem.vehicles[i].maxLoad > 0) {  // 修改判断方式
            cout << "无人机, 载重: " << problem.vehicles[i].maxLoad 
                 << ", 电池容量: " << problem.vehicles[i].fuel;
        } else {
            cout << "车辆";
        }
        cout << "):\n";
        
        // 输出配送路径序列
        cout << "  任务序列: ";
        for (const auto &task : allPaths[i].first) {
            cout << task << " ";
        }
        // 输出每个任务的完成时间
        cout << "\n  任务完成时间: ";
        for (const auto &time : allPaths[i].second) {
            cout << time << " ";
        }
        cout << "\n";
    }

    return 0;
}
