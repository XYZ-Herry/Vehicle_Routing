#ifndef SOLVER_H
#define SOLVER_H

#include "common.h"
#include <vector>
#include <utility>
#include <unordered_map>

// 优化所有车辆的路径 - 修改返回类型为<车辆ID, <路径, 时间>>
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> optimizeAllPaths(
    const DeliveryProblem& problem,
    const std::vector<std::pair<int, int>>& vehicleTaskAssignments);

// 求解静态配送问题 - 同样修改返回类型
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> solveStaticProblem(
    DeliveryProblem& problem);

// 计算总完成时间和总成本 - 修改参数类型
std::pair<double, double> calculateTotalTimeAndCost(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& allPaths);

// 求解动态配送问题 - 修改参数和返回类型
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> solveDynamicProblem(
    DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    double staticMaxTime);

// 识别需要重新安排的任务 - 修改参数类型
void identifyTasksForRescheduling(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    double staticMaxTime,
    std::vector<int>& delayedTasks,
    std::vector<int>& newTasks);

#endif // SOLVER_H 