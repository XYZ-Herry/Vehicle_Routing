#ifndef SOLVER_H
#define SOLVER_H

#include "common.h"
#include <vector>
#include <utility>
#include <unordered_map>

// 优化所有车辆的路径
std::vector<std::pair<std::vector<int>, std::vector<double>>> optimizeAllPaths(
    const DeliveryProblem& problem,
    const std::vector<std::pair<int, int>>& vehicleTaskAssignments);

// 求解静态配送问题
std::vector<std::pair<std::vector<int>, std::vector<double>>> solveStaticProblem(
    DeliveryProblem& problem);

// 计算总完成时间和总成本
std::pair<double, double> calculateTotalTimeAndCost(
    const DeliveryProblem& problem,
    const std::vector<std::pair<std::vector<int>, std::vector<double>>>& allPaths);

#endif // SOLVER_H 