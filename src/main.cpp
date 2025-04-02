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
    
    // 验证静态阶段结果的正确性
    bool staticCenterValid = validateStaticVehicleCenter(problem, staticPaths);
    auto [staticPathValid, staticErrorMsg] = validateStaticPathLegality(problem, staticPaths);
    
    if (!staticCenterValid) {
        cout << "静态阶段车辆中心分配验证失败!" << endl;
    }
    
    if (!staticPathValid) {
        cout << "静态阶段路径合法性验证失败:" << endl;
        cout << staticErrorMsg << endl;
    }
    
    // 解决动态配送问题
    auto dynamicPaths = solveDynamicProblem(problem, staticPaths, staticMaxTime);
    
    // 输出动态阶段配送结果详情
    Print_DeliveryResults(problem, dynamicPaths);

    // 验证动态阶段结果的正确性
    bool dynamicCenterValid = validateDynamicVehicleCenter(
        problem, staticPaths, dynamicPaths, staticMaxTime);
    
    auto [dynamicPathValid, dynamicErrorMsg] = validateDynamicPathLegality(
        problem, dynamicPaths, newTasks);
    
    if (!dynamicCenterValid) {
        cout << "动态阶段车辆中心分配验证失败!" << endl;
    }
    
    if (!dynamicPathValid) {
        cout << "动态阶段路径合法性验证失败:" << endl;
        cout << dynamicErrorMsg << endl;
    }
    
    // 输出验证结果总结
    cout << "\n===== 路径验证结果 =====\n";
    cout << "静态阶段车辆中心分配: " << (staticCenterValid ? "通过" : "失败") << endl;
    cout << "静态阶段路径合法性: " << (staticPathValid ? "通过" : "失败") << endl;
    cout << "动态阶段车辆中心分配: " << (dynamicCenterValid ? "通过" : "失败") << endl;
    cout << "动态阶段路径合法性: " << (dynamicPathValid ? "通过" : "失败") << endl;
    
    return 0;
}
