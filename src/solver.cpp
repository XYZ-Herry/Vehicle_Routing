#include "solver.h"
#include "task_assigner.h"
#include "path_optimizer.h"
#include "genetic_algorithm.h"
#include <algorithm>
#include <iostream>
#include <iomanip>


using std::vector;
using std::pair;
using std::make_pair;
using std::unordered_map;
using std::cout;
using std::endl;
using std::max;

// 优化所有车辆的配送路径
vector<pair<vector<int>, vector<double>>> optimizeAllPaths(
    DeliveryProblem& problem, 
    const vector<pair<int, int>>& assignments)
{
    vector<pair<vector<int>, vector<double>>> allPaths(problem.vehicles.size());
    
    // 整理每个车辆的任务
    unordered_map<int, vector<int>> vehicleAssignments;
    for (const auto& [vehicleId, taskId] : assignments) {
        vehicleAssignments[vehicleId].push_back(taskId);
    }
    
    // 为每个车辆优化路径
    for (size_t i = 0; i < problem.vehicles.size(); ++i) {
        if (vehicleAssignments.count(i) > 0) {
            // 优化路径
            auto path = optimizePathForVehicle(
                vehicleAssignments[i], 
                problem.tasks, 
                problem.vehicles[i],
                problem.network);
            
            // 计算完成时间
            auto completionTimes = calculateCompletionTimes(
                path, 
                problem.tasks,
                problem.vehicles[i],
                problem.network);
                
            allPaths[i] = make_pair(path, completionTimes);
        }
    }
    
    return allPaths;
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

// 计算总完成时间和总成本
pair<double, double> calculateTotalTimeAndCost(
    const DeliveryProblem& problem,
    const vector<pair<vector<int>, vector<double>>>& allPaths) 
{
    double totalCompletionTime = 0.0;
    double totalCost = 0.0;
    
    // 遍历所有车辆的路径
    for (int i = 0; i < problem.vehicles.size(); ++i) {
        if (!allPaths[i].first.empty() && !allPaths[i].second.empty()) {
            // 更新最大完成时间
            double vehicleCompletionTime = allPaths[i].second.back();
            totalCompletionTime = max(totalCompletionTime, vehicleCompletionTime);
            
            // 计算运送成本（只与货物数量有关）
            // 路径中的任务点数量（不包括起点和终点的配送中心）
            int taskCount = allPaths[i].first.size() - 2;
            totalCost += problem.vehicles[i].cost * taskCount;
        }
    }
    
    return {totalCompletionTime, totalCost};
} 