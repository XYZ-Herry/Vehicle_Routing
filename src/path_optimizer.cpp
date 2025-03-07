#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <limits>
#include <cmath>

using std::vector;
using std::numeric_limits;

// 使用最近邻法优化车辆的配送路径
vector<int> optimizePathForVehicle(
    const vector<int> &assignedTasks, 
    const vector<TaskPoint> &tasks, 
    const Vehicle &vehicle,
    const RouteNetwork &network,
    const vector<DistributionCenter> &centers)
{
    if (assignedTasks.empty()) {
        return {vehicle.centerId};  // 如果没有分配任务，直接返回配送中心
    }

    vector<int> path;                    // 优化后的配送路径
    vector<bool> visited(tasks.size());  // 记录任务点是否已访问
    int centerId = vehicle.centerId;     // 车辆所属的配送中心ID
    double currentLoad = 0.0;            // 当前载重
    double remainingFuel = vehicle.fuel; // 剩余电量
    int currentPos = centerId;           // 当前位置
    
    // 查找配送中心坐标
    double centerX = 0, centerY = 0;
    for (const auto& center : centers) {
        if (center.id == centerId) {
            centerX = center.x;
            centerY = center.y;
            break;
        }
    }
    
    // 从配送中心开始
    path.push_back(centerId);
    
    // 使用最近邻法构建路径
    while (true) {
        double minDist = numeric_limits<double>::max();
        int nextTask = -1;
        bool allTasksNeedReturn = true;  // 假设所有未访问任务点都需要返回配送中心

        // 寻找最近的未访问任务点
        for (int taskId : assignedTasks) {
            if (!visited[taskId]) {
                // 计算到达该点的距离和时间
                double dist;
                if (currentPos == centerId) {
                    // 从配送中心到任务点
                    dist = sqrt(pow(tasks[taskId].x - centerX, 2) + pow(tasks[taskId].y - centerY, 2));
                } else {
                    // 从任务点到任务点
                    dist = getDistance(tasks[currentPos], tasks[taskId], network, vehicle.maxLoad > 0);
                }
                
                double timeNeeded = dist / vehicle.speed;
                
                bool canVisitDirectly = true;  // 默认可以直接访问
                
                // 如果是无人机，检查约束
                if (vehicle.maxLoad > 0) {
                    // 检查载重约束
                    if (currentLoad + tasks[taskId].weight > vehicle.maxLoad) {
                        canVisitDirectly = false;  // 需要返回卸货
                    }

                    // 检查电量约束
                    if (timeNeeded > remainingFuel) {
                        canVisitDirectly = false; // 电量不足
                    }
                    
                    // 计算完成任务后返回配送中心所需电量
                    double returnDist = sqrt(pow(tasks[taskId].x - centerX, 2) + pow(tasks[taskId].y - centerY, 2));
                    double returnTime = returnDist / vehicle.speed;
                    
                    // 如果无法安全返回
                    if (timeNeeded + returnTime > remainingFuel) {
                        canVisitDirectly = false;  // 无法安全返回
                    }
                }
                
                // 如果至少有一个任务点可以直接访问，则不是所有任务点都需要返回
                if (canVisitDirectly) {
                    allTasksNeedReturn = false;
                    
                    // 如果该点距离更近，且满足约束，更新为候选点
                    if (dist < minDist) {
                        minDist = dist;
                        nextTask = taskId;
                    }
                }
            }
        }

        // 如果所有未访问任务点都需要返回配送中心，且还有未访问任务点
        if (allTasksNeedReturn && anyTaskUnvisited(visited, assignedTasks)) {
            // 添加返回配送中心的路径点
            path.push_back(centerId);
            
            // 重置状态
            currentLoad = 0.0;
            remainingFuel = vehicle.fuel;
            currentPos = centerId;
            continue;  // 重新寻找任务点
        }

        // 如果找到了下一个任务点
        if (nextTask != -1) {
            visited[nextTask] = true;
            path.push_back(nextTask);
            
            // 更新状态
            if (vehicle.maxLoad > 0) {
                currentLoad += tasks[nextTask].weight;
                remainingFuel -= minDist / vehicle.speed;
            }
            currentPos = nextTask;
        } else {
            // 没有找到满足条件的任务点，退出循环
            break;
        }
    }

    // 检查是否所有任务点都已访问
    bool allVisited = true;
    for (int taskId : assignedTasks) {
        if (!visited[taskId]) {
            allVisited = false;
            break;
        }
    }
    
    if (!allVisited) {
        return vector<int>();  // 返回空解，表示不可行
    }

    // 如果当前不在配送中心，添加返回配送中心的路径
    if (currentPos != centerId) {
        path.push_back(centerId);
    }
    
    return path;
}

// 辅助函数：检查是否还有未访问的任务点
bool anyTaskUnvisited(const vector<bool>& visited, const vector<int>& tasks) {
    for (int taskId : tasks) {
        if (!visited[taskId]) {
            return true;
        }
    }
    return false;
}

// 计算某辆车路径上每个任务点的完成时间
vector<double> calculateCompletionTimes(
    const vector<int> &path, 
    const vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const RouteNetwork &network,
    const vector<DistributionCenter> &centers)
{
    if (path.empty()) {
        return {0.0};
    }
    
    vector<double> completionTimes;
    double currentTime = 0.0;
    double currentLoad = 0.0;
    int currentPos = vehicle.centerId;
    
    // 查找配送中心坐标
    double centerX = 0, centerY = 0;
    for (const auto& center : centers) {
        if (center.id == vehicle.centerId) {
            centerX = center.x;
            centerY = center.y;
            break;
        }
    }
    
    // 计算每个任务点的完成时间（注意路径中可能包含多次返回配送中心）
    for (size_t i = 1; i < path.size(); ++i) {
        int nextPos = path[i];
        
        // 计算从当前位置到下一位置的距离和时间
        double dist;
        if (currentPos == vehicle.centerId) {
            // 从配送中心到任务点
            dist = sqrt(pow(tasks[nextPos].x - centerX, 2) + pow(tasks[nextPos].y - centerY, 2));
        } else if (nextPos == vehicle.centerId) {
            // 从任务点返回配送中心
            dist = sqrt(pow(tasks[currentPos].x - centerX, 2) + pow(tasks[currentPos].y - centerY, 2));
        } else {
            // 从任务点到任务点
            dist = getDistance(tasks[currentPos], tasks[nextPos], network, vehicle.maxLoad > 0);
        }
        
        currentTime += dist / vehicle.speed;
        
        // 如果下一个位置是配送中心，重置载重
        if (nextPos == vehicle.centerId) {
            currentLoad = 0.0;
        } 
        // 如果下一个位置是任务点，添加完成时间
        else {
            currentLoad += tasks[nextPos].weight;
            completionTimes.push_back(currentTime);
        }
        
        currentPos = nextPos;
    }
    
    // 确保至少有一个完成时间
    if (completionTimes.empty()) {
        completionTimes.push_back(0.0);
    }
    
    return completionTimes;
}