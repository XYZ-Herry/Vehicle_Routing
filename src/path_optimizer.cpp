#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <limits>
#include <cmath>

using std::vector;
using std::numeric_limits;

// 使用最近邻法优化车辆的配送路径
vector<int> optimizePathForVehicle(const vector<int> &assignedTasks, const vector<TaskPoint> &tasks, const Vehicle &vehicle, const RouteNetwork &network)
{
    if (assignedTasks.empty()) {
        return {vehicle.centerId};  // 如果没有分配任务，直接返回配送中心
    }
    vector<int> path;                    // 优化后的配送路径
    vector<bool> visited(tasks.size());  // 记录任务点是否已访问
    int centerId = vehicle.centerId;     // 车辆所属的配送中心ID
    double minDist;                      // 到最近任务点的距离
    int nextTask;                        // 下一个要访问的任务点ID
    
    // 从配送中心开始
    path.push_back(centerId);
    
    // 使用最近邻法构建路径
    while (path.size() < assignedTasks.size() + 1)
    {
        int last = path.back();
        minDist = numeric_limits<double>::max();
        nextTask = -1;

        // 寻找最近的未访问任务点
        for (int taskId : assignedTasks)
        {
            if (!visited[taskId])
            {
                double dist = getDistance(tasks[last], tasks[taskId], 
                                       network, vehicle.maxLoad > 0);
                if (dist < minDist)
                {
                    minDist = dist;
                    nextTask = taskId;
                }
            }
        }

        if (nextTask != -1)
        {
            visited[nextTask] = true;
            path.push_back(nextTask);
        }
        else
        {
            break;
        }
    }

    // 返回配送中心
    path.push_back(centerId);
    return path;
}

// 计算某辆车路径上每个任务点的完成时间
vector<double> calculateCompletionTimes(
    const vector<int> &path, 
    const vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const RouteNetwork &network)
{
    if (path.empty()) {
        return {0.0};  // 如果路径为空，返回0时间
    }
    
    vector<double> completionTimes;
    double currentTime = 0.0;
    
    // 计算每个任务点的完成时间
    for (size_t i = 1; i < path.size(); ++i) {
        int currentTask = path[i-1];
        int nextTask = path[i];
        
        // 计算行驶时间
        double travelDist = getDistance(tasks[currentTask], tasks[nextTask], 
                                     network, vehicle.maxLoad > 0);
        
        if (travelDist == std::numeric_limits<double>::max()) {
            printf("出错了");
        }
        
        double travelTime = travelDist / vehicle.speed;
        currentTime += travelTime;
        
        // 如果是额外需求点，需要考虑到达时间约束
        // if (tasks[nextTask].arrivalTime > 0) {
        //     currentTime = std::max(currentTime, tasks[nextTask].arrivalTime);
        // }
        
        completionTimes.push_back(currentTime);
    }
    
    // 确保至少有一个完成时间
    if (completionTimes.empty()) {
        completionTimes.push_back(0.0);
    }
    
    return completionTimes;
}