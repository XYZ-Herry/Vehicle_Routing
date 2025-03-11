#include "solver.h"
#include "task_assigner.h"
#include "path_optimizer.h"
#include "genetic_algorithm.h"
#include "dynamic_genetic.h"
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
    vector<pair<vector<int>, vector<double>>> allPaths(problem.vehicles.size());
    
    unordered_map<int, vector<int>> vehicleToTasks;
    for (const auto &assignment : vehicleTaskAssignments) {
        vehicleToTasks[assignment.first].push_back(assignment.second);
    }
    
    for (const auto &pair : vehicleToTasks) {
        int vehicleId = pair.first;
        const auto &assignedTasks = pair.second;
        
        vector<int> path = optimizePathForVehicle(
            assignedTasks, 
            problem.tasks, 
            problem.vehicles[vehicleId], 
            problem);
        
        vector<double> completionTimes = calculateCompletionTimes(
            path, 
            problem.tasks, 
            problem.vehicles[vehicleId], 
            problem);
        
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
vector<pair<vector<int>, vector<double>>> solveStaticProblem(DeliveryProblem& problem)
{
    assignTasksToCenters(problem);
    
    auto vehicleTaskAssignments = geneticAlgorithm(
        problem,
        100,    // 种群大小
        100,    // 迭代次数
        0.1,    // 变异率
        problem.timeWeight
    );
    
    auto allPaths = optimizeAllPaths(problem, vehicleTaskAssignments);
    
    // 计算最晚完成时间
    double latestCompletionTime = 0.0;
    for (const auto& [path, completionTimes] : allPaths) {
        if (!completionTimes.empty()) {
            latestCompletionTime = std::max(latestCompletionTime, completionTimes.back());
        }
    }
    
    // 输出最晚完成时间
    cout << "静态阶段所有任务的最晚完成时间: " << latestCompletionTime << " 小时" << endl;
    
    return allPaths;
}

// 求解动态配送问题
vector<pair<vector<int>, vector<double>>> solveDynamicProblem(
    DeliveryProblem& problem,
    const vector<pair<vector<int>, vector<double>>>& staticPaths,
    double staticMaxTime)
{
    cout << "开始动态求解..." << endl;
    // 1. 找出受高峰期影响而超时的任务和新增任务
    vector<int> delayedTasks;     // 受高峰期影响超时的任务
    vector<int> newTasks;         // 新增任务
    
    cout << "识别需要重新安排的任务..." << endl;
    // 找出需要重新安排的任务
    identifyTasksForRescheduling(problem, staticPaths, staticMaxTime, delayedTasks, newTasks);
    
    cout << "识别完成，延迟任务: " << delayedTasks.size() << ", 新任务: " << newTasks.size() << endl;
    
    // 如果没有需要重新安排的任务，直接返回静态结果
    if (delayedTasks.empty() && newTasks.empty()) {
        cout << "没有任务需要重新安排，保持静态方案。" << endl;
        return staticPaths;
    }
    
    cout << "开始动态遗传算法..." << endl;
    // 2. 使用动态遗传算法重新安排任务
    auto dynamicAssignments = dynamicGeneticAlgorithm(
        problem,
        staticPaths,
        delayedTasks,
        newTasks,
        100,    // 种群大小
        100,    // 迭代次数
        0.1,    // 变异率
        problem.timeWeight,
        staticMaxTime
    );
    
    cout << "动态遗传算法完成，开始优化路径..." << endl;
    // 3. 优化所有路径
    auto dynamicPaths = optimizeDynamicPaths(problem, dynamicAssignments, staticPaths);
    
    cout << "动态路径优化完成" << endl;
    return dynamicPaths;
}

// 识别需要重新安排的任务
void identifyTasksForRescheduling(
    const DeliveryProblem& problem,
    const vector<pair<vector<int>, vector<double>>>& staticPaths,
    double staticMaxTime,
    vector<int>& delayedTasks,
    vector<int>& newTasks)
{
    // 添加新增任务
    for (size_t i = problem.initialDemandCount; i < problem.tasks.size(); ++i) {
        newTasks.push_back(i);
    }
    
    // 检查每个车辆的路径，找出在高峰期会延迟的任务
    for (size_t vehicleId = 0; vehicleId < staticPaths.size(); ++vehicleId) {
        const auto& [path, staticTimes] = staticPaths[vehicleId];
        if (path.empty()) continue;
        
        // 重新计算考虑高峰期的完成时间
        vector<double> dynamicTimes = calculateCompletionTimes(
            path, problem.tasks, problem.vehicles[vehicleId], problem, true);
        
        // 比较每个任务点，找出延迟的任务
        for (size_t i = 0; i < std::min(staticTimes.size(), dynamicTimes.size()); ++i) {
            if (dynamicTimes[i] > staticMaxTime && staticTimes[i] <= staticMaxTime) {
                // 找到路径中对应的任务点索引
                if (i + 1 < path.size()) {  // 确保索引在范围内
                    int taskIndex = path[i + 1]; // +1 因为第一个是配送中心
                    delayedTasks.push_back(taskIndex);
                }
            }
        }
    }
} 