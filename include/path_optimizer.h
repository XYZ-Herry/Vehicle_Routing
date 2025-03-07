#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H

#include "common.h"
#include <vector>

// 使用最近邻法优化车辆的配送路径
std::vector<int> optimizePathForVehicle(
    const std::vector<int> &assignedTasks, 
    const std::vector<TaskPoint> &tasks, 
    const Vehicle &vehicle,
    const RouteNetwork &network,
    const std::vector<DistributionCenter> &centers);

// 计算某辆车路径上每个任务点的完成时间
std::vector<double> calculateCompletionTimes(
    const std::vector<int> &path, 
    const std::vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const RouteNetwork &network,
    const std::vector<DistributionCenter> &centers);

// 辅助函数：检查是否还有未访问的任务点
bool anyTaskUnvisited(
    const std::vector<bool>& visited,
    const std::vector<int>& tasks);

#endif