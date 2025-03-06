#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H

#include "common.h"
#include <vector>

// 使用最近邻法优化车辆的配送路径
std::vector<int> optimizePathForVehicle(
    const std::vector<int> &assignedTasks, 
    const std::vector<TaskPoint> &tasks, 
    const Vehicle &vehicle,
    const RouteNetwork &network);

// 计算路径上每个任务点的完成时间
std::vector<double> calculateCompletionTimes(
    const std::vector<int> &path, 
    const std::vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const RouteNetwork &network);

#endif