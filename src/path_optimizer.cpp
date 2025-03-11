#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <unordered_map>
#include <iterator>

using std::vector;
using std::numeric_limits;
using std::unordered_map;
using std::pair;

// 使用最近邻法优化车辆的配送路径
vector<int>  optimizePathForVehicle(
    const vector<int> &assignedTasks, 
    const vector<TaskPoint> &tasks, 
    const Vehicle &vehicle,
    const DeliveryProblem& problem)
{
    if (assignedTasks.empty()) {
        return {vehicle.centerId};
    }

    vector<int> path;
    vector<bool> visited(tasks.size());
    int centerId = vehicle.centerId;
    double currentLoad = 0.0;
    double remainingFuel = vehicle.fuel;
    int currentPos = centerId;
    
    path.push_back(centerId);
    
    while (true) {
        double minDist = numeric_limits<double>::max();
        int nextTask = -1;
        bool allTasksNeedReturn = true;

        for (int taskIndex : assignedTasks) {
            if (!visited[taskIndex]) {
                // 计算到达该点的距离和时间
                double dist = getDistance(
                    currentPos == centerId ? centerId : tasks[currentPos].id,
                    tasks[taskIndex].id,
                    problem,
                    vehicle.maxLoad > 0
                );
                
                double timeNeeded = dist / vehicle.speed;
                bool canVisitDirectly = true;
                
                if (vehicle.maxLoad > 0) {
                    if (currentLoad + tasks[taskIndex].weight > vehicle.maxLoad) {
                        canVisitDirectly = false;
                    }

                    if (timeNeeded > remainingFuel) {
                        canVisitDirectly = false;
                    }
                    
                    // 计算返回配送中心的距离
                    double returnDist = getDistance(
                        tasks[taskIndex].id,
                        centerId,
                        problem,
                        true
                    );
                    double returnTime = returnDist / vehicle.speed;
                    
                    if (timeNeeded + returnTime > remainingFuel) {
                        canVisitDirectly = false;
                    }
                }
                
                if (canVisitDirectly) {
                    allTasksNeedReturn = false;
                    if (dist < minDist) {
                        minDist = dist;
                        nextTask = taskIndex;
                    }
                }
            }
        }

        if (allTasksNeedReturn && anyTaskUnvisited(visited, assignedTasks)) {
            path.push_back(centerId);
            currentLoad = 0.0;
            remainingFuel = vehicle.fuel;
            currentPos = centerId;
            continue;
        }

        if (nextTask != -1) {
            visited[nextTask] = true;
            path.push_back(nextTask);
            
            if (vehicle.maxLoad > 0) {
                currentLoad += tasks[nextTask].weight;
                remainingFuel -= minDist / vehicle.speed;
            }
            currentPos = nextTask;
        } else {
            break;
        }
    }

    bool allVisited = true;
    for (int taskIndex : assignedTasks) {
        if (!visited[taskIndex]) {
            allVisited = false;
            break;
        }
    }
    
    if (!allVisited) {
        return vector<int>();
    }

    if (currentPos != centerId) {
        path.push_back(centerId);
    }
    
    return path;
}

// 辅助函数：检查是否还有未访问的任务点
bool anyTaskUnvisited(const vector<bool>& visited, const vector<int>& tasks) {
    for (int taskIndex : tasks) {
        if (!visited[taskIndex]) {
            return true;
        }
    }
    return false;
}

// 根据时间和路段判断是否处于高峰期，返回速度系数
double getSpeedFactor(double currentTime, int fromId, int toId, const DeliveryProblem& problem) {
    // 转换为小时数
    double hour = currentTime;
    
    // 获取该路段的高峰期系数
    auto it1 = problem.network.peakFactors.find(fromId);
    if (it1 != problem.network.peakFactors.end()) {
        auto it2 = it1->second.find(toId);
        if (it2 != it1->second.end()) {
            // 判断是否在早高峰（7:00-9:00）
            if (hour >= 7.0 && hour <= 9.0) {
                return it2->second.first;  // 早高峰系数
            }
            
            // 判断是否在晚高峰（17:00-18:00）
            if (hour >= 17.0 && hour <= 18.0) {
                return it2->second.second;  // 晚高峰系数
            }
        }
    }
    
    // 非高峰期或未找到特定路段系数
    return 1.0;
}

// 计算某辆车路径上每个任务点的完成时间
vector<double> calculateCompletionTimes(
    const vector<int> &path, 
    const vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const DeliveryProblem& problem,
    bool considerTraffic)
{
    if (path.empty()) {
        return {0.0};
    }
    
    vector<double> completionTimes;
    double currentTime = 0.0;  // 初始时间为0点
    double currentLoad = 0.0;
    int currentPos = vehicle.centerId;
    
    for (size_t i = 1; i < path.size(); ++i) {
        int nextPos = path[i];
        
        // 获取当前位置和下一位置的实际ID
        int fromId = currentPos == vehicle.centerId ? vehicle.centerId : tasks[currentPos].id;
        int toId = nextPos == vehicle.centerId ? vehicle.centerId : tasks[nextPos].id;
        
        // 计算从当前位置到下一位置的距离
        double dist = getDistance(fromId, toId, problem, vehicle.maxLoad > 0);
        
        // 仅动态阶段考虑高峰期因素
        double speedFactor = 1.0;  // 默认无交通影响
        if (considerTraffic) {
            speedFactor = getSpeedFactor(currentTime, fromId, toId, problem);
        }
        
        // 计算行驶时间
        double travelTime = dist / (vehicle.speed * speedFactor);
        currentTime += travelTime;
        
        if (nextPos == vehicle.centerId) {
            currentLoad = 0.0;
        } else {
            currentLoad += tasks[nextPos].weight;
            completionTimes.push_back(currentTime);
        }
        
        currentPos = nextPos;
    }
    
    if (completionTimes.empty()) {
        completionTimes.push_back(0.0);
    }
    
    return completionTimes;
}

// 优化动态阶段的所有路径
vector<std::pair<vector<int>, vector<double>>> optimizeDynamicPaths(
    const DeliveryProblem& problem,
    const vector<pair<int, int>>& dynamicAssignments,
    const vector<pair<vector<int>, vector<double>>>& staticPaths)
{
    // 不再使用静态路径，直接按遗传算法的分配构建
    
    // 按车辆收集所有分配的任务
    unordered_map<int, vector<int>> vehicleTasks;
    for (const auto& [vehicleId, taskId] : dynamicAssignments) {
        vehicleTasks[vehicleId].push_back(taskId);
    }
    
    // 为每个车辆创建优化路径
    vector<std::pair<vector<int>, vector<double>>> dynamicPaths(problem.vehicles.size());
    
    // 为每个车辆优化路径
    for (size_t vehicleId = 0; vehicleId < problem.vehicles.size(); ++vehicleId) {
        // 获取车辆所属配送中心ID
        int centerId = problem.vehicles[vehicleId].centerId;
        
        // 检查该车辆是否有分配的任务
        if (vehicleTasks.count(vehicleId) && !vehicleTasks[vehicleId].empty()) {
            // 提取分配给该车辆的所有任务
            const auto& tasks = vehicleTasks[vehicleId];
            
            // 优化路径
            vector<int> path = optimizePathForVehicle(
                tasks, problem.tasks, problem.vehicles[vehicleId], problem);
            
            // 如果路径优化成功
            if (!path.empty()) {
                // 计算考虑高峰期的完成时间
                vector<double> times = calculateCompletionTimes(
                    path, problem.tasks, problem.vehicles[vehicleId], problem, true);
                
                // 保存路径和完成时间
                dynamicPaths[vehicleId] = {path, times};
            } else {
                // 路径优化失败，创建只有起点和终点的空路径
                dynamicPaths[vehicleId] = {{centerId, centerId}, {0.0}};
            }
        } else {
            // 该车辆没有任务，创建只有起点和终点的空路径
            dynamicPaths[vehicleId] = {{centerId, centerId}, {0.0}};
        }
    }
    
    return dynamicPaths;
}