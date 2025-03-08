#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

#include "common.h"
#include <vector>
#include <utility>

// 计算解的适应度
double calculateFitness(
    const std::vector<int>& solution,
    const std::vector<int>& centerTasks,
    const std::vector<TaskPoint>& tasks,
    const std::vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight);

// 遗传算法主函数
std::vector<std::pair<int, int>> geneticAlgorithm(
    const DeliveryProblem& problem,
    int populationSize,
    int generations,
    double mutationRate,
    double timeWeight);

#endif