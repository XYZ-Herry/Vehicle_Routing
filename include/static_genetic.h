#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include "common.h"
#include <vector>
#include <utility>

// 计算解的适应度
double calculateFitness(
    const std::vector<int>& solution,        // 存储车辆ID
    const std::vector<int>& centerTaskIds,   // 存储任务ID
    const std::vector<TaskPoint>& tasks,
    const std::vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight);

// 静态遗传算法主函数
std::vector<std::pair<int, int>> Static_GeneticAlgorithm(
    const DeliveryProblem& problem,
    int populationSize,
    int generations,
    double mutationRate,
    double timeWeight);

#endif