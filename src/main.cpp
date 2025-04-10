#include <iostream>
#include <vector>
#include <limits>
#include <string>
#include <cmath>
#include "common.h"
#include "solver.h"
#include "path_validator.h"

using std::vector;
using std::pair;
using std::cout;
using std::endl;
using std::string;

// 声明全局变量
vector<int> delayedTasks, newTasks;

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
    
    // 打印初始信息
    printInitialInfo(problem);
    
    
    // 求解静态配送问题
    cout << "\n========== 静态阶段求解 ==========" << endl;
    
    auto staticPaths = solveStaticProblem(problem);
    cout << "\n========== 配送中心车辆和任务分配 ==========" << endl;
    printCenterAssignments(problem);
    
    // 计算静态阶段的最大完成时间
    double staticMaxTime = 0.0;
    for (const auto& [vehicleId, pair] : staticPaths) {
        const auto& [path, completionTimes] = pair;
        if (!completionTimes.empty()) {
            staticMaxTime = std::max(staticMaxTime, completionTimes[completionTimes.size() - 2]);
        }
    }
    // 输出静态阶段结果
    Print_DeliveryResults(problem, staticPaths);
    
    // 解决动态配送问题
    auto dynamicPaths = solveDynamicProblem(problem, staticPaths, staticMaxTime);
    
    // 输出动态阶段配送结果详情
    Print_DeliveryResults(problem, dynamicPaths);
    
    auto [dynamicPathValid, dynamicErrorMsg] = validateDynamicPathLegality(
        problem, dynamicPaths, newTasks);

    
    // 输出验证结果总结
    cout << "\n===== 路径验证结果 =====\n";
    
    // 验证所有路径的合法性和完整性
    auto [isValid, errorMessage] = validateAllPaths(problem, staticPaths, dynamicPaths, staticMaxTime, newTasks);
    
    if (isValid) {
        cout << "路径验证通过，所有约束条件满足！" << endl;
    } else {
        cout << "路径验证失败，请检查详细错误信息！" << endl;
        std::cerr << errorMessage << endl;  // 输出详细错误信息
    }
    
    return 0;
}
