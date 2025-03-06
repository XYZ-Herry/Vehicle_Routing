#ifndef SOLVER_H
#define SOLVER_H

#include "common.h"
#include <vector>
#include <utility>
#include <unordered_map>

// 优化所有车辆的配送路径
std::vector<std::pair<std::vector<int>, std::vector<double>>> optimizeAllPaths(
    DeliveryProblem& problem, 
    const std::vector<std::pair<int, int>>& assignments);

// 完整的静态问题求解函数
std::vector<std::pair<std::vector<int>, std::vector<double>>> solveStaticProblem(
    DeliveryProblem& problem);

// 计算总完成时间和总成本
std::pair<double, double> calculateTotalTimeAndCost(
    const DeliveryProblem& problem,
    const std::vector<std::pair<std::vector<int>, std::vector<double>>>& allPaths);

#endif // SOLVER_H 