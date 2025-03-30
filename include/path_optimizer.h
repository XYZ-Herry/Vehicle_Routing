#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H

#include <vector>
#include "common.h"

// 使用最近邻法优化车辆的配送路径
std::vector<int> optimizePathForVehicle(
    const std::vector<int> &assignedTaskIds,  // 任务ID列表
    const std::vector<TaskPoint> &tasks, 
    const Vehicle &vehicle,
    const DeliveryProblem& problem);

// 辅助函数：检查是否还有未访问的任务点
bool anyTaskUnvisited(const std::vector<bool>& visited, const std::vector<int>& taskIds);

// 根据时间和路段判断是否处于高峰期，返回速度系数
double getSpeedFactor(double currentTime, int fromId, int toId, const DeliveryProblem& problem);

// 计算某辆车路径上每个任务点的完成时间
std::vector<double> calculateCompletionTimes(
    const std::vector<int> &path, 
    const std::vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const DeliveryProblem& problem,
    bool considerTraffic = false);

// 优化动态阶段的所有路径 - 更新返回类型
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> optimizeDynamicPaths(
    const DeliveryProblem& problem,
    const std::vector<std::pair<int, int>>& dynamicAssignments);

// 动态阶段为车辆优化考虑时间约束的路径
std::pair<std::vector<int>, std::vector<double>> Dynamic_OptimizePathForVehicle(
    const std::vector<int> &assignedTaskIds,
    const std::vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const DeliveryProblem& problem);

// 考虑车辆协同的无人机路径规划 - 修改返回类型
std::pair<std::vector<int>, std::vector<double>> optimizeDronePathWithVehicles(
    const std::vector<int>& taskIds,
    const std::vector<TaskPoint>& tasks,
    const Vehicle& drone,
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<int, double>>& taskVisitInfo);

// 计算从一个点到另一个点需要的时间
double calculateTimeNeeded(
    int currentId,      // 当前点id
    int destId,         // 目的点id
    double currentTime, // 当前时间
    const Vehicle& vehicle,
    const DeliveryProblem& problem,
    bool considerTraffic = false,  // 是否考虑高峰期
    bool isDrone = false);  // 是否是无人机

#endif // PATH_OPTIMIZER_H