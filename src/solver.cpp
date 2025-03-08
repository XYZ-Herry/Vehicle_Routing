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
    
    return optimizeAllPaths(problem, vehicleTaskAssignments);
} 