#ifndef DYNAMIC_GENETIC_H
#define DYNAMIC_GENETIC_H

#include "common.h"
#include <vector>
#include <utility>
#include <unordered_map>

// 计算动态阶段的适应度
double calculateDynamicFitness(
    const std::vector<int>& solution,
    const std::vector<int>& tasksToAssign,
    const std::unordered_map<int, int>& staticAssignments,
    const std::vector<TaskPoint>& tasks,
    const std::vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight,
    double staticMaxTime);

// 动态阶段的遗传算法
std::vector<std::pair<int, int>> dynamicGeneticAlgorithm(
    const DeliveryProblem& problem,
    const std::vector<std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    const std::vector<int>& delayedTasks,
    const std::vector<int>& newTasks,
    int populationSize,
    int generations,
    double mutationRate,
    double timeWeight,
    double staticMaxTime);

#endif // DYNAMIC_GENETIC_H 