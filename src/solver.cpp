#include "solver.h"
#include "task_assigner.h"
#include "path_optimizer.h"
#include "static_genetic.h"
#include "dynamic_genetic.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cmath>

using std::vector;
using std::pair;
using std::make_pair;
using std::unordered_map;
using std::unordered_set;
using std::cout;
using std::endl;
using std::max;

// 优化所有车辆的路径 - 修改为返回<车辆ID, <路径, 时间>>的形式
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> static_optimizeAllPaths(
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
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    
    for (const auto& [vehicleId, pathData] : allPaths) {
        const auto& [path, times] = pathData;
        
        // 跳过空路径
        if (path.empty() || times.empty()) {
            continue;
        }
        
        // 使用时间数组的倒数第二个元素作为该车辆的完成时间
        // (如果数组长度至少为2)
        if (times.size() >= 2) {
            double vehicleCompletionTime = times[times.size() - 2];
            maxCompletionTime = std::max(maxCompletionTime, vehicleCompletionTime);
        }
        
        // 计算实际任务数量（不包括配送中心）
        int actualTaskCount = 0;
        for (int pointId : path) {
            if (problem.centerIds.count(pointId) == 0) {
                // 如果不是配送中心ID，则是任务点
                actualTaskCount++;
            }
        }
        
        // 查找车辆信息以获取成本
        if (problem.vehicleIdToIndex.count(vehicleId)) {
            int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
            totalCost += actualTaskCount * problem.vehicles[vehicleIndex].cost;
        }
    }
    
    return {maxCompletionTime, totalCost};
}

// 求解静态配送问题
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> solveStaticProblem(DeliveryProblem& problem)
{
    assignTasksToCenters(problem);
    
    auto vehicleTaskAssignments = Static_GeneticAlgorithm(
        problem,
        DeliveryProblem::DEFAULT_POPULATION_SIZE,
        DeliveryProblem::DEFAULT_GENERATIONS,
        DeliveryProblem::DEFAULT_MUTATION_RATE,
        problem.timeWeight
    );
    
    auto allPaths = static_optimizeAllPaths(problem, vehicleTaskAssignments);
    
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
    // 输出延迟任务ID
    if (!delayedTasks.empty()) {
        cout << "延迟任务ID: ";
        for (size_t i = 0; i < delayedTasks.size(); ++i) {
            if (i > 0) cout << ", ";
            cout << delayedTasks[i];
        }
        cout << endl;
    }
    
    cout << "新增任务数量: " << newTasks.size() << endl;
    // 输出新增任务ID
    if (!newTasks.empty()) {
        cout << "新增任务ID: ";
        for (size_t i = 0; i < newTasks.size(); ++i) {
            if (i > 0) cout << ", ";
            cout << newTasks[i];
        }
        cout << endl;
    }
    
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
    return optimizeDynamicPaths(problem, assignments);
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
    cout << "--------------------------------" << endl;
    std::cout << "考虑高峰期后的静态阶段路径时间：" << std::endl;
    // 检查每个车辆的路径，找出在高峰期会延迟的任务
    for (const auto& [vehicleId, pair] : staticPaths) {
        const auto& [path, staticTimes] = pair;
        if (path.empty()) continue;
        
        // 获取车辆信息
        int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
        const Vehicle& vehicle = problem.vehicles[vehicleIndex];
        bool isDrone = (vehicle.maxLoad > 0);
        
        // 重新计算考虑高峰期的完成时间
        vector<double> dynamicTimes = calculateCompletionTimes(
            path, problem.tasks, vehicle, problem, true);
        
        // 输出该车辆在高峰期的路径时间
        std::cout << (isDrone ? "无人机" : "车辆") << " #" << vehicleId << " (" 
                  << (isDrone ? "无人机" : "卡车") << ") 的路径: ";
        
        // 输出路径，区分中心和任务点
        for (size_t i = 0; i < path.size(); ++i) {
            bool isCenter = problem.centerIds.count(path[i]) > 0;
            std::cout << (isCenter ? "中心#" : "任务#") << path[i];
            if (i < path.size() - 1) std::cout << " -> ";
        }
        std::cout << "\n";
        
        // 输出完成时间
        std::cout << "完成时间: ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << " " << std::fixed << std::setprecision(3) << dynamicTimes[i] << "h";
            
            // 如果有延迟，标记出来
            if (i < staticTimes.size() && dynamicTimes[i] > staticTimes[i] + 0.001) {
                std::cout << "(延迟)";
            }
            
            if (i < path.size() - 1) std::cout << ",";
        }
        std::cout << "\n\n";
        
        // 比较每个任务点，找出延迟的任务
        for (size_t i = 0; i < std::min(staticTimes.size(), dynamicTimes.size()); ++i) {
            // 判断是否为任务点（不是配送中心）
            if (problem.centerIds.count(path[i]) == 0) {
                // 如果动态时间超过了静态最大时间，但静态时间没有超过，则标记为延迟任务
                if (dynamicTimes[i] > staticMaxTime && staticTimes[i] <= staticMaxTime) {
                    delayedTasks.push_back(path[i]); // 直接使用path[i]作为任务ID
                }
            }
        }
    }
} 