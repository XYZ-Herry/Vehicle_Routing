#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <cmath>
#include <unordered_map>
#include <iterator>

using std::vector;
using std::numeric_limits;
using std::unordered_map;
using std::pair;

// 使用最近邻法优化车辆的配送路径
vector<int> optimizePathForVehicle(
    const vector<int> &assignedTaskIds,  // 任务ID列表
    const vector<TaskPoint> &tasks, 
    const Vehicle &vehicle,
    const DeliveryProblem& problem)
{
    // 如果没有分配任务，只返回起点和终点
    if (assignedTaskIds.empty()) {
        return {vehicle.centerId, vehicle.centerId};
    }
    
    int centerId = vehicle.centerId;
    
    // 检查是否为无人机（无人机有最大载重限制）
    bool isDrone = (vehicle.maxLoad > 0);
    
    if (!isDrone) {
        // 普通车辆使用原来的最近邻算法
        vector<int> path;
        vector<bool> visited(assignedTaskIds.size(), false);
        
        // 从配送中心开始
        path.push_back(centerId);
        int currentPos = centerId;
        
        // 根据距离选择下一个访问点，直到所有点都被访问
        while (anyTaskUnvisited(visited, assignedTaskIds)) {
            double minDistance = std::numeric_limits<double>::max();
            int nextIndex = -1;
            int nextId = -1;
            
            for (size_t i = 0; i < assignedTaskIds.size(); i++) {
                if (!visited[i]) {
                    int taskId = assignedTaskIds[i];
                    
                    double distance = getDistance(
                        currentPos, 
                        taskId, 
                        problem,
                        false); // 非无人机
                    
                    if (distance < minDistance) {
                        minDistance = distance;
                        nextIndex = i;
                        nextId = taskId;
                    }
                }
            }
            
            if (nextIndex != -1) {
                visited[nextIndex] = true;
                path.push_back(nextId);
                currentPos = nextId;
            } else {
                break;  // 无法找到下一个点，结束
            }
        }
        
        // 回到配送中心
        path.push_back(centerId);
        
        return path;
    } else {
        // 无人机路径规划，考虑电量和载重约束
        vector<int> path;
        vector<bool> visited(assignedTaskIds.size(), false);
        
        // 从配送中心开始
        path.push_back(centerId);
        int currentPos = centerId;
        
        // 无人机初始状态
        double currentBattery = vehicle.maxfuel; // 满电量
        double currentLoad = 0.0; // 初始载重为0
        double maxProcessLoad = 0.0; // 一次行程中的最大载重
        
        // 当还有未访问的任务点时继续循环
        while (anyTaskUnvisited(visited, assignedTaskIds)) {
            double minDistance = std::numeric_limits<double>::max();
            int nextIndex = -1;
            int nextId = -1;
            
            // 寻找满足约束的最近任务点
            for (size_t i = 0; i < assignedTaskIds.size(); i++) {
                if (!visited[i]) {
                    int taskId = assignedTaskIds[i];
                    int taskIndex = problem.taskIdToIndex.at(taskId);
                    const TaskPoint& task = tasks[taskIndex];
                    
                    // 计算到该任务点的距离
                    double distanceToTask = getDistance(currentPos, taskId, problem, true);
                    
                    // 计算从该任务点到配送中心的距离
                    double distanceToCenter = getDistance(taskId, centerId, problem, true);
                    
                    // 计算所需电量
                    double batteryNeededToTask = distanceToTask / vehicle.speed;
                    double batteryNeededToCenter = distanceToCenter / vehicle.speed;
                    double totalBatteryNeeded = batteryNeededToTask + batteryNeededToCenter;
                    
                    // 检查电量约束
                    if (totalBatteryNeeded > currentBattery) {
                        continue; // 电量不足，跳过该任务点
                    }
                    
                    // 检查载重约束
                    bool isPickup = (task.pickweight > 0);
                    
                    if (isPickup) {
                        // 取货点：检查当前载重 + 取货重量是否超过最大载重
                        if (currentLoad + task.pickweight > vehicle.maxLoad) {
                            continue; // 超过载重限制，跳过该任务点
                        }
                    } else {
                        // 送货点：检查过程最大载重 + 送货重量是否超过最大载重
                        if (maxProcessLoad + task.sendWeight > vehicle.maxLoad) {
                            continue; // 超过载重限制，跳过该任务点
                        }
                    }
                    
                    // 如果满足所有约束，并且距离小于当前最小距离，更新最近任务点
                    if (distanceToTask < minDistance) {
                        minDistance = distanceToTask;
                        nextIndex = i;
                        nextId = taskId;
                    }
                }
            }
            
            // 如果找到下一个可行的任务点
            if (nextIndex != -1) {
                int taskIndex = problem.taskIdToIndex.at(nextId);
                const TaskPoint& nextTask = tasks[taskIndex];
                double distanceToNext = getDistance(currentPos, nextId, problem, true);
                
                // 更新状态
                visited[nextIndex] = true;
                path.push_back(nextId);
                currentPos = nextId;
                
                // 更新电量（距离/速度 = 消耗的小时数）
                currentBattery -= distanceToNext / vehicle.speed;
                
                // 根据任务类型更新载重
                bool isPickup = (nextTask.pickweight > 0);
                if (isPickup) {
                    currentLoad += nextTask.pickweight;
                    maxProcessLoad = std::max(maxProcessLoad, currentLoad);
                } else {
                }
            } else {
                // 如果没有找到可行的下一个任务点，返回配送中心
                double distanceToCenter = getDistance(currentPos, centerId, problem, true);
                
                // 检查是否有足够电量返回
                if (currentBattery >= distanceToCenter / vehicle.speed) {
                    path.push_back(centerId);
                    // 回到配送中心后重置状态
                    currentPos = centerId;
                    currentBattery = vehicle.maxfuel; // 充满电
                    currentLoad = 0.0; // 卸货
                    maxProcessLoad = 0.0; // 重置过程最大载重
                } else {
                    // 电量不足以返回，异常情况
                    std::cerr << "警告: 无人机 #" << vehicle.id << " 电量不足以返回配送中心！" << std::endl;
                    return {}; // 返回空路径表示规划失败
                }
            }
        }
        
        // 如果当前不在配送中心，添加返回配送中心的路径
        if (currentPos != centerId) {
            double distanceToCenter = getDistance(currentPos, centerId, problem, true);
            
            // 检查是否有足够电量返回
            if (currentBattery >= distanceToCenter / vehicle.speed) {
                path.push_back(centerId);
            } else {
                // 电量不足以返回，异常情况
                std::cerr << "警告: 无人机 #" << vehicle.id << " 电量不足以返回配送中心！" << std::endl;
                return {}; // 返回空路径表示规划失败
            }
        }
        
        return path;
    }
}

// 辅助函数：检查是否还有未访问的任务点
bool anyTaskUnvisited(const vector<bool>& visited, const vector<int>& taskIds) {
    for (size_t i = 0; i < visited.size(); i++) {
        if (!visited[i]) {
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
    const vector<int> &path,  // 路径中存储的是任务ID，不是索引
    const vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const DeliveryProblem& problem,
    bool considerTraffic)
{
    if (path.empty()) {
        return {0.0};
    }
    
    vector<double> completionTimes;
    double currentTime = 0.0;
    double currentLoad = 0.0;
    int currentPosId = path[0];  // 第一个位置是中心ID
    
    for (size_t i = 1; i < path.size(); ++i) {
        int nextPosId = path[i];  // 下一个位置ID
        
        // 直接使用ID计算距离
        double dist = getDistance(currentPosId, nextPosId, problem, vehicle.maxLoad > 0);
        
        // 考虑高峰期因素
        double speedFactor = 1.0;
        if (considerTraffic) {
            speedFactor = getSpeedFactor(currentTime, currentPosId, nextPosId, problem);
        }
        
        double travelTime = dist / (vehicle.speed * speedFactor);
        currentTime += travelTime;
        completionTimes.push_back(currentTime);
        currentPosId = nextPosId;
    }
    
    if (completionTimes.empty()) {
        completionTimes.push_back(0.0);
    }
    
    return completionTimes;
}

// 优化动态阶段的所有路径 - 修改为返回<车辆ID, <路径, 时间>>的形式
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> optimizeDynamicPaths(
    const DeliveryProblem& problem,
    const std::vector<std::pair<int, int>>& dynamicAssignments // (车辆ID, 任务ID)对
)
{
    // 按车辆ID收集所有分配的任务ID
    std::unordered_map<int, std::vector<int>> vehicleIdToTaskIds; // 车辆ID -> 任务ID列表
    for (const auto& assignment : dynamicAssignments) {
        int vehicleId = assignment.first;
        int taskId = assignment.second;
        vehicleIdToTaskIds[vehicleId].push_back(taskId);
    }
    
    /*
        在这个函数里面实现车机协同的功能，目前只是各走各的，也没有考虑额外需求点时间的限制
    */

    
    // 为每个车辆创建优化路径
    std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> dynamicPaths;
    
    // 为每个车辆优化路径
    for (size_t vehicleIndex = 0; vehicleIndex < problem.vehicles.size(); ++vehicleIndex) {
        int vehicleId = problem.vehicles[vehicleIndex].id;
        int centerId = problem.vehicles[vehicleIndex].centerId;
        
        // 检查该车辆是否有分配的任务
        if (vehicleIdToTaskIds.count(vehicleId) && !vehicleIdToTaskIds[vehicleId].empty()) {
            // 提取分配给该车辆的所有任务ID
            const auto& assignedTaskIds = vehicleIdToTaskIds[vehicleId];
            
            // 使用最近邻算法优化路径
            std::vector<int> path = optimizePathForVehicle(
                assignedTaskIds,  // 传递任务ID
                problem.tasks,
                problem.vehicles[vehicleIndex], //传递车辆
                problem
            );
            
            // 计算完成时间（考虑交通）
            std::vector<double> completionTimes = calculateCompletionTimes(
                path,  // path中存储的是任务点的ID
                problem.tasks,
                problem.vehicles[vehicleIndex], 
                problem,
                true  // 考虑交通影响
            );
            
            dynamicPaths[vehicleId] = {path, completionTimes};
        } 
        else {
            // 创建只包含起点和终点的路径
            std::vector<int> path = {centerId, centerId};
            std::vector<double> times = {0.0, 0.0};
            dynamicPaths[vehicleId] = {path, times};
        }
    }
    
    return dynamicPaths;  // 返回<车辆ID, <路径(任务ID序列), 时间>>的形式
}
