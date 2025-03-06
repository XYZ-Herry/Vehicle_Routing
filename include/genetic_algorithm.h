#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include "common.h"
#include <vector>
#include <utility>
#include "path_optimizer.h"

// 使用遗传算法优化每个配送中心内的车辆任务分配
std::vector<std::pair<int, int>> geneticAlgorithm(
    const std::vector<TaskPoint>& tasks,
    const std::vector<Vehicle>& vehicles,
    const std::vector<DistributionCenter>& centers,
    int populationSize,
    int generations,
    double mutationRate,
    const RouteNetwork& network,
    double timeWeight);

// 计算解的适应度
double calculateFitness(const std::vector<int>& solution,
                       const std::vector<int>& centerTasks,
                       const std::vector<TaskPoint>& tasks,
                       const std::vector<Vehicle>& vehicles,
                       const RouteNetwork& network,
                       double timeWeight);

#endif