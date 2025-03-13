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

// 优化所有车辆的路径 - 修改为返回<车辆ID, <路径, 时间>>的形式
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> optimizeAllPaths(
    const DeliveryProblem& problem,
    const std::vector<std::pair<int, int>>& vehicleTaskAssignments)  // (车辆ID, 任务ID)对
{
    std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> allPaths;
    
    // 按车辆ID收集任务ID
    std::unordered_map<int, std::vector<int>> vehicleIdToTaskIds;  // 车辆ID -> 任务ID列表
    for (const auto &assignment : vehicleTaskAssignments) {
        int vehicleId = assignment.first;
        int taskId = assignment.second;
        vehicleIdToTaskIds[vehicleId].push_back(taskId);
    }
    
    // 按车辆ID优化路径
    for (const auto &pair : vehicleIdToTaskIds) {
        int vehicleId = pair.first;
        const auto &assignedTaskIds = pair.second;
        
        // 将车辆ID转换为索引
        if (problem.vehicleIdToIndex.count(vehicleId) > 0) {
            int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
            
            // 使用最近邻算法优化路径
            std::vector<int> path = optimizePathForVehicle(
                assignedTaskIds,  // 传递任务ID列表
                problem.tasks, 
                problem.vehicles[vehicleIndex], 
                problem);
            
            // 计算完成时间
            std::vector<double> completionTimes = calculateCompletionTimes(
                path,  // path中存储的是任务点ID
                problem.tasks, 
                problem.vehicles[vehicleIndex], 
                problem);
            
            // 以车辆ID为键存储路径
            allPaths[vehicleId] = {path, completionTimes};
        }
    }
    
    return allPaths;  // 返回<车辆ID, <路径, 时间>>的形式
}

// 计算总完成时间和总成本
pair<double, double> calculateTotalTimeAndCost(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& allPaths)
{
    double totalTime = 0.0;
    double totalCost = 0.0;
    
    for (const auto &pair : allPaths) {
        const auto &[path, completionTimes] = pair.second;
        if (!path.empty()) {
            // 更新最大完成时间
            if (!completionTimes.empty()) {
                totalTime = std::max(totalTime, completionTimes.back());
            }
            
            // 计算运送成本（减去起点和终点的配送中心）
            int actualTaskCount = path.size() - 2;
            totalCost += actualTaskCount * problem.vehicles[pair.first].cost;
        }
    }
    
    return {totalTime, totalCost};
}

// 求解静态配送问题
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> solveStaticProblem(DeliveryProblem& problem)
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
    for (const auto& [vehicleId, pair] : allPaths) {
        const auto& [path, completionTimes] = pair;
        if (!completionTimes.empty()) {
            latestCompletionTime = std::max(latestCompletionTime, completionTimes.back());
        }
    }
    
    // 输出最晚完成时间
    cout << "静态阶段所有任务的最晚完成时间: " << latestCompletionTime << " 小时" << endl;
    
    return allPaths;
}

// 求解动态配送问题
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> solveDynamicProblem(
    DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    double staticMaxTime)
{
    // 识别需要重新调度的任务
    vector<int> delayedTasks, newTasks;
    identifyTasksForRescheduling(problem, staticPaths, staticMaxTime, delayedTasks, newTasks);
    
    cout << "延迟任务数量: " << delayedTasks.size() << endl;
    cout << "新增任务数量: " << newTasks.size() << endl;
    
    if (delayedTasks.empty() && newTasks.empty()) {
        cout << "没有需要重新调度的任务，直接使用静态解决方案" << endl;
        return staticPaths;
    }
    
    // 使用改进的动态遗传算法分配任务
    vector<pair<int, int>> assignments = dynamicGeneticAlgorithm(
        problem, staticPaths, delayedTasks, newTasks,
        100,  // 种群大小
        50,   // 迭代次数
        0.1,  // 变异率
        problem.timeWeight,  // 时间权重
        staticMaxTime);
    
    if (assignments.empty()) {
        cout << "动态优化失败，继续使用静态解决方案" << endl;
        return staticPaths;
    }
    
    // 优化所有车辆的路径
    return optimizeDynamicPaths(problem, assignments, staticPaths);
}

// 识别需要重新安排的任务
void identifyTasksForRescheduling(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    double staticMaxTime,
    vector<int>& delayedTasks,
    vector<int>& newTasks)
{
    // 添加新增任务
    for (size_t i = problem.initialDemandCount; i < problem.tasks.size(); ++i) {
        newTasks.push_back(problem.tasks[i].id);  // 添加任务ID而不是索引
    }
    
    // 检查每个车辆的路径，找出在高峰期会延迟的任务
    for (const auto& [vehicleId, pair] : staticPaths) {
        const auto& [path, staticTimes] = pair;
        if (path.empty()) continue;
        
        // 重新计算考虑高峰期的完成时间
        vector<double> dynamicTimes = calculateCompletionTimes(
            path, problem.tasks, problem.vehicles[vehicleId], problem, true);
        
        // 比较每个任务点，找出延迟的任务
        for (size_t i = 0; i < std::min(staticTimes.size(), dynamicTimes.size()); ++i) {
            if (dynamicTimes[i] > staticMaxTime && staticTimes[i] <= staticMaxTime) {
                // 找到路径中对应的任务点ID (不是索引)
                if (i + 1 < path.size()) {  // 确保索引在范围内
                    int taskId = path[i + 1]; // +1 因为第一个是配送中心
                    delayedTasks.push_back(taskId); // 这里存的是任务ID
                }
            }
        }
    }
} 