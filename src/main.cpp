#include <iostream>
#include <vector>
#include <limits>
#include <string>
#include <cmath>
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
    
    // 求解静态配送问题
    cout << "\n========== 静态阶段求解 ==========" << endl;
    auto staticPaths = solveStaticProblem(problem);//staticPaths 是一个包含所有车辆路径的向量，每个路径是一个包含任务ID的向量，以及每个任务的完成时间。
    
    // 计算静态阶段的最大完成时间
    double staticMaxTime = 0.0;
    for (const auto& [vehicleId, pathData] : staticPaths) {
        const auto& [path, times] = pathData;
        if (!times.empty()) {
            staticMaxTime = std::max(staticMaxTime, times.back());
        }
    }
    
    // 输出静态阶段配送结果详情
    printDeliveryResults(problem, staticPaths);
    
    // 求解动态阶段问题
    cout << "\n========== 动态阶段求解 ==========" << endl;
    cout << "静态阶段最大完成时间 T = " << staticMaxTime << " 小时" << endl;
    
    auto dynamicPaths = solveDynamicProblem(problem, staticPaths, staticMaxTime);
    
    // 输出动态阶段配送结果详情
    cout << "\n========== 动态阶段结果 ==========" << endl;
    printDeliveryResults(problem, dynamicPaths);
    
    return 0;
}
