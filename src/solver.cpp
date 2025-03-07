#include "solver.h"
#include "task_assigner.h"
#include "path_optimizer.h"
#include "genetic_algorithm.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cmath>

using std::vector;
using std::pair;
using std::make_pair;
using std::unordered_map;
using std::cout;
using std::endl;
using std::max;

// 优化所有车辆的路径
vector<pair<vector<int>, vector<double>>> optimizeAllPaths(
    const DeliveryProblem& problem,
    const vector<pair<int, int>>& vehicleTaskAssignments)
{
    // 初始化结果数组
    vector<pair<vector<int>, vector<double>>> allPaths(problem.vehicles.size());
    
    // 整理每辆车分配到的任务
    unordered_map<int, vector<int>> vehicleToTasks;
    for (const auto &assignment : vehicleTaskAssignments) {
        int vehicleId = assignment.first;
        int taskId = assignment.second;
        
        // 将任务添加到对应车辆的分配列表
        vehicleToTasks[vehicleId].push_back(taskId);
    }
    
    // 为每辆车优化路径
    for (const auto &pair : vehicleToTasks) {
        int vehicleId = pair.first;
        const auto &assignedTasks = pair.second;
        
        // 调用路径优化函数，注意添加 centers 参数
        vector<int> path = optimizePathForVehicle(
            assignedTasks, 
            problem.tasks, 
            problem.vehicles[vehicleId], 
            problem.network,
            problem.centers);  // 添加 centers 参数
        
        // 计算完成时间
        vector<double> completionTimes = calculateCompletionTimes(
            path, 
            problem.tasks, 
            problem.vehicles[vehicleId], 
            problem.network,
            problem.centers);  // 添加 centers 参数
        
        // 保存结果
        allPaths[vehicleId] = {path, completionTimes};
    }
    
    return allPaths;
}

// 计算总完成时间和总成本
pair<double, double> calculateTotalTimeAndCost(
    const DeliveryProblem& problem,
    const vector<pair<vector<int>, vector<double>>>& allPaths)
{
    double totalTime = 0.0;
    double totalCost = 0.0;
    
    for (size_t i = 0; i < allPaths.size(); ++i) {
        if (!allPaths[i].first.empty()) {
            // 更新最大完成时间
            if (!allPaths[i].second.empty()) {
                totalTime = std::max(totalTime, allPaths[i].second.back());
            }
            
            // 计算运送成本（减去起点和终点的配送中心）
            int actualTaskCount = allPaths[i].first.size() - 2;
            totalCost += actualTaskCount * problem.vehicles[i].cost;
        }
    }
    
    return {totalTime, totalCost};
}

// 求解静态配送问题
vector<pair<vector<int>, vector<double>>> solveStaticProblem(DeliveryProblem& problem) {
    // 第一阶段：将任务点分配到最近的配送中心
    assignTasksToCenters(problem);
    
    // 第二阶段：使用遗传算法优化每个配送中心内的车辆任务分配
    auto vehicleTaskAssignments = geneticAlgorithm(
        problem.tasks,
        problem.vehicles,
        problem.centers,
        100,  // 种群大小
        100,  // 迭代次数
        0.1,  // 变异率
        problem.network,
        problem.timeWeight  // 传递时间权重
    );
    
    // 第三阶段：优化每个车辆的配送路径
    auto allPaths = optimizeAllPaths(problem, vehicleTaskAssignments);
    
    // 计算总完成时间和总成本
    auto [totalTime, totalCost] = calculateTotalTimeAndCost(problem, allPaths);
    
    // 输出总体结果
    cout << "\n==================== 求解结果 ====================" << endl;
    cout << std::fixed << std::setprecision(2);
    cout << "总时间: " << totalTime << " h" << endl;
    cout << "总成本: " << totalCost << " 元" << endl;
    cout << "目标值: " << (problem.timeWeight * totalTime + (1.0 - problem.timeWeight) * totalCost) << endl;
    
    // 输出详细信息
    cout << "\n================ 车辆配送方案 =================" << endl;
    for (size_t i = 0; i < allPaths.size(); ++i) {
        if (!allPaths[i].first.empty()) {
            cout << "\n-------------------------------------------" << endl;
            if (problem.vehicles[i].maxLoad > 0) {
                cout << "无人机 #" << i << endl;
                cout << "载重量: " << problem.vehicles[i].maxLoad << " kg" << endl;
                cout << "飞行速度: " << problem.vehicles[i].speed << " km/h" << endl;
                cout << "续航时间: " << problem.vehicles[i].fuel << " h" << endl;
            } else {
                cout << "车辆 #" << i << endl;
                cout << "行驶速度: " << problem.vehicles[i].speed << " km/h" << endl;
            }
            
            // 输出配送路径
            cout << "配送路径: ";
            for (size_t j = 0; j < allPaths[i].first.size(); ++j) {
                cout << allPaths[i].first[j];
                if (j < allPaths[i].first.size() - 1) cout << " → ";
            }
            cout << endl;
            
            // 输出任务数量和成本
            int actualTaskCount = allPaths[i].first.size() - 2;  // 减去起点和终点（都是配送中心）
            cout << "配送任务: " << actualTaskCount << " 件" << endl;
            cout << "运输成本: " << (actualTaskCount * problem.vehicles[i].cost) << " 元" << endl;
            
            // 输出完成时间
            if (!allPaths[i].second.empty()) {
                cout << "完成时间: " << allPaths[i].second.back() << " h" << endl;
            }
        }
    }
    cout << "===========================================" << endl;
    
    return allPaths;
} 