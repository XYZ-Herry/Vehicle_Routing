#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <cmath>
#include <set>
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
                    std::cerr << "警告: 无人机 #" << vehicle.id << " 无解" << std::endl;
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
            // 判断是否在早高峰
            if (hour >= DeliveryProblem::MORNING_PEAK_START && hour <= DeliveryProblem::MORNING_PEAK_END) {
                return it2->second.first;  // 早高峰系数
            }
            
            // 判断是否在晚高峰
            if (hour >= DeliveryProblem::EVENING_PEAK_START && hour <= DeliveryProblem::EVENING_PEAK_END) {
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
        return {};
    }
    
    vector<double> completionTimes(path.size(), 0.0);
    double currentTime = 0.0;
    
    // 遍历路径中的每一段
    for (size_t i = 0; i < path.size() - 1; i++) {
        int fromId = path[i];
        int toId = path[i+1];
        
        // 计算距离
        double distance = getDistance(fromId, toId, problem, vehicle.maxLoad > 0);
        
        // 如果考虑交通，车辆调整速度
        double speed = vehicle.speed;
        if (considerTraffic && vehicle.maxLoad > 0) {
            double speedFactor = getSpeedFactor(currentTime, fromId, toId, problem);
            speed *= speedFactor;
        }
        
        // 计算当前段的行驶时间
        double travelTime = distance / speed;
        
        // 更新当前时间
        currentTime += travelTime;
        
        // 记录到达toId的时间
        completionTimes[i+1] = currentTime;
        
    }
    
    return completionTimes;
}

// 优化动态阶段的所有路径 - 实现车机协同
std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> optimizeDynamicPaths(
    const DeliveryProblem& problem,
    const std::vector<std::pair<int, int>>& dynamicAssignments // (车辆ID, 任务ID)对
)
{
    // 按车辆ID收集任务
    std::unordered_map<int, std::vector<int>> vehicleIdToTaskIds;
    for (const auto& assignment : dynamicAssignments) {
        vehicleIdToTaskIds[assignment.first].push_back(assignment.second);
    }
    
    // 结果存储
    std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>> dynamicPaths;
    
    // 任务点访问信息：<任务点ID, <车辆ID, 到达时间>>
    std::unordered_map<int, std::pair<int, double>> taskVisitInfo;
    
    // 阶段1：先规划普通车辆路径
    for (const auto& vehicle : problem.vehicles) {
        int vehicleId = vehicle.id;
        
        // 跳过无人机
        if (vehicle.maxLoad > 0) continue;
        
        // 检查是否有分配的任务
        if (vehicleIdToTaskIds.count(vehicleId) && !vehicleIdToTaskIds[vehicleId].empty()) {
            // 规划路径
            std::vector<int> path = optimizePathForVehicle(
                vehicleIdToTaskIds[vehicleId],
                problem.tasks,
                vehicle,
                problem
            );
            
            // 计算时间 - 车辆需要考虑交通
            std::vector<double> times = calculateCompletionTimes(
                path, problem.tasks, vehicle, problem, true
            );
            
            // 保存路径和时间
            dynamicPaths[vehicleId] = {path, times};
            
            // 记录任务点访问信息
            for (size_t i = 0; i < path.size(); ++i) {
                int pointId = path[i];
                if (problem.centerIds.count(pointId) == 0) {
                    taskVisitInfo[pointId] = {vehicleId, times[i]};
                }
            }
        } else {
            // 空路径
            dynamicPaths[vehicleId] = {{vehicle.centerId, vehicle.centerId}, {0.0, 0.0}};
        }
    }
    
    // 阶段2：规划无人机路径，考虑与车辆协同
    for (const auto& drone : problem.vehicles) {
        int droneId = drone.id;
        
        // 只处理无人机
        if (drone.maxLoad <= 0) continue;
        
        // 检查是否有分配的任务
        if (vehicleIdToTaskIds.count(droneId) && !vehicleIdToTaskIds[droneId].empty()) {
            // 使用协同算法规划无人机路径
            auto [path, times] = optimizeDronePathWithVehicles(
                vehicleIdToTaskIds[droneId],
                problem.tasks,
                drone,
                problem,
                taskVisitInfo
            );
            
            if (!path.empty()) {
                // 直接使用返回的时间而不重新计算
                dynamicPaths[droneId] = {path, times};
            } else {
                // 协同失败，使用标准算法
                path = optimizePathForVehicle(
                    vehicleIdToTaskIds[droneId],
                    problem.tasks,
                    drone,
                    problem
                );
                
                if (!path.empty()) {
                    std::vector<double> times = calculateCompletionTimes(
                        path, problem.tasks, drone, problem, false
                    );
                    dynamicPaths[droneId] = {path, times};
                } else {
                    // 空路径
                    dynamicPaths[droneId] = {{drone.centerId, drone.centerId}, {0.0, 0.0}};
                }
            }
        } else {
            // 空路径
            dynamicPaths[droneId] = {{drone.centerId, drone.centerId}, {0.0, 0.0}};
        }
    }
    
    return dynamicPaths;
}

// 考虑车辆协同的无人机路径规划
std::pair<std::vector<int>, std::vector<double>> optimizeDronePathWithVehicles(
    const std::vector<int>& taskIds,
    const std::vector<TaskPoint>& tasks,
    const Vehicle& drone,
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<int, double>>& taskVisitInfo
) {
    if (taskIds.empty()) {
        return {{drone.centerId, drone.centerId}, {0.0, 0.0}};
    }
    
    std::vector<int> path;
    std::vector<double> times;  // 添加时间记录
    std::vector<bool> visited(taskIds.size(), false);
    int currentPos = drone.centerId;
    path.push_back(currentPos);
    times.push_back(0.0);  // 初始时间为0
    
    double currentBattery = drone.maxfuel;
    double currentLoad = 0.0;
    double maxProcessLoad = 0.0;
    double currentTime = 0.0;
    
    while (anyTaskUnvisited(visited, taskIds)) {
        double minDistance = std::numeric_limits<double>::max();
        int nextIndex = -1;
        int nextId = -1;
        
        // 寻找满足约束的下一个任务点
        for (size_t i = 0; i < taskIds.size(); i++) {
            if (visited[i]) continue;
            
            int taskId = taskIds[i];
            int taskIndex = problem.taskIdToIndex.at(taskId);
            const TaskPoint& task = tasks[taskIndex];
            
            // 检查载重约束
            bool isPickup = (task.pickweight > 0);
            if (isPickup) {
                if (currentLoad + task.pickweight > drone.maxLoad) continue;
            } else {
                if (maxProcessLoad + task.sendWeight > drone.maxLoad) continue;
            }
            
            // 计算到任务点的距离和电量需求
            double distanceToTask = getDistance(currentPos, taskId, problem, true);
            double batteryNeededToTask = distanceToTask / drone.speed;
            
            // 检查电量是否足够到达任务点
            if (batteryNeededToTask > currentBattery) continue;
            
            // 检查从任务点是否有可行的返回点
            bool canReturn = false;
            
            // 首先检查是否可以返回原配送中心
            double distanceToOriginalCenter = getDistance(taskId, drone.centerId, problem, true);
            double batteryToOriginalCenter = distanceToOriginalCenter / drone.speed;
            
            if (batteryNeededToTask + batteryToOriginalCenter <= currentBattery) {
                canReturn = true;
            }
            
            // 如果不能返回原配送中心，则检查是否可以返回车辆经过的任务点
            if (!canReturn) {
                for (const auto& [visitTaskId, info] : taskVisitInfo) {
                    // 跳过当前检查的任务点
                    if (visitTaskId == taskId) continue;
                    
                    double distanceToVisitPoint = getDistance(taskId, visitTaskId, problem, true);
                    double batteryToVisitPoint = distanceToVisitPoint / drone.speed;
                    double arrivalTime = currentTime + batteryNeededToTask + batteryToVisitPoint;
                    
                    // 如果无人机能到达该点，且在车辆到达前抵达
                    if (batteryNeededToTask + batteryToVisitPoint <= currentBattery && 
                        arrivalTime < info.second) {
                        canReturn = true;
                        break; // 只要找到一个可行的返回点即可
                    }
                }
            }
            
            // 如果找不到可返回的点，跳过该任务点
            if (!canReturn) continue;
            
            // 更新最近的下一个任务点
            if (distanceToTask < minDistance) {
                minDistance = distanceToTask;
                nextIndex = i;
                nextId = taskId;
            }
        }
        
        // 如果找到下一个可行任务点
        if (nextIndex != -1) {
            // 访问该任务点
            visited[nextIndex] = true;
            path.push_back(nextId);
            
            // 更新状态
            double travelTime = minDistance / drone.speed;
            currentTime += travelTime;
            currentBattery -= travelTime;
            currentPos = nextId;
            
            // 更新载重
            int taskIndex = problem.taskIdToIndex.at(nextId);
            const TaskPoint& task = tasks[taskIndex];
            bool isPickup = (task.pickweight > 0);
            
            if (isPickup) {
                currentLoad += task.pickweight;
                maxProcessLoad = std::max(maxProcessLoad, currentLoad);
            } else {

            }
            
            // 记录到达时间
            times.push_back(currentTime);
        } else {
            // 无法找到下一个任务点，需要返回某个点
            int bestReturnPoint = drone.centerId;
            double minReturnTime = std::numeric_limits<double>::max(); // 最早可以完成返回的时间
            
            // 首先检查是否能返回原配送中心
            double distanceToCenter = getDistance(currentPos, drone.centerId, problem, true);
            double batteryNeeded = distanceToCenter / drone.speed;
            double returnTime = currentTime + distanceToCenter / drone.speed;
            
            if (batteryNeeded <= currentBattery) {
                minReturnTime = returnTime;
                bestReturnPoint = drone.centerId;
            }
            
            // 寻找可到达的车辆经过点
            for (const auto& [visitTaskId, info] : taskVisitInfo) {
                double distance = getDistance(currentPos, visitTaskId, problem, true);
                double batteryNeeded = distance / drone.speed;
                double droneArrivalTime = currentTime + distance / drone.speed;
                double vehicleArrivalTime = info.second;
                
                // 计算实际可完成返回的时间（需要等待车辆到达）
                double actualReturnTime = std::max(droneArrivalTime, vehicleArrivalTime);
                
                // 如果无人机能到达该点，且能在车辆到达前抵达，且最终返回时间更早
                if (batteryNeeded <= currentBattery && 
                    droneArrivalTime < vehicleArrivalTime && 
                    actualReturnTime < minReturnTime) {
                    minReturnTime = actualReturnTime;
                    bestReturnPoint = visitTaskId;
                }
            }
            
            // 如果找到可返回的点
            if (minReturnTime < std::numeric_limits<double>::max()) {
                path.push_back(bestReturnPoint);
                
                // 计算返回时间
                double distance = getDistance(currentPos, bestReturnPoint, problem, true);
                double flyingTime = distance / drone.speed;
                currentBattery -= flyingTime;
                
                // 更新当前位置
                currentPos = bestReturnPoint;
                
                // 如果是车辆访问的任务点，需要等待车辆到达
                if (bestReturnPoint != drone.centerId && taskVisitInfo.count(bestReturnPoint) > 0) {
                    double vehicleArrivalTime = taskVisitInfo.at(bestReturnPoint).second;
                    // 更新到达时间（考虑等待车辆）
                    currentTime = std::max(currentTime + flyingTime, vehicleArrivalTime);
                } else {
                    // 直接返回配送中心，不需要等待
                    currentTime += flyingTime;
                }
                
                // 在返回点充电和卸货
                currentBattery = drone.maxfuel;
                currentLoad = 0.0;
                maxProcessLoad = 0.0;
                
                // 记录到达时间
                times.push_back(currentTime);
            } else {
                // 无法找到返回点，规划失败
                return {{drone.centerId, drone.centerId}, {0.0, 0.0}};
            }
        }
    }
    
    // 如果最后不在配送中心，选择返回某个点
    if (currentPos != drone.centerId) {
        int bestReturnPoint = drone.centerId;
        double minReturnTime = std::numeric_limits<double>::max(); // 最早可以完成返回的时间
        
        // 首先检查是否能返回原配送中心
        double distanceToCenter = getDistance(currentPos, drone.centerId, problem, true);
        double batteryNeeded = distanceToCenter / drone.speed;
        double returnTime = currentTime + distanceToCenter / drone.speed;
        
        if (batteryNeeded <= currentBattery) {
            minReturnTime = returnTime;
            bestReturnPoint = drone.centerId;
        }
        
        // 寻找可到达的车辆经过点
        for (const auto& [visitTaskId, info] : taskVisitInfo) {
            double distance = getDistance(currentPos, visitTaskId, problem, true);
            double batteryNeeded = distance / drone.speed;
            double droneArrivalTime = currentTime + distance / drone.speed;
            double vehicleArrivalTime = info.second;
            
            // 计算实际可完成返回的时间（需要等待车辆到达）
            double actualReturnTime = std::max(droneArrivalTime, vehicleArrivalTime);
            
            // 如果无人机能到达该点，且能在车辆到达前抵达，且最终返回时间更早
            if (batteryNeeded <= currentBattery && 
                droneArrivalTime < vehicleArrivalTime && 
                actualReturnTime < minReturnTime) {
                minReturnTime = actualReturnTime;
                bestReturnPoint = visitTaskId;
            }
        }
        
        // 如果找到可返回的点
        if (minReturnTime < std::numeric_limits<double>::max()) {
            path.push_back(bestReturnPoint);
            
            // 计算返回时间
            double distance = getDistance(currentPos, bestReturnPoint, problem, true);
            double flyingTime = distance / drone.speed;
            currentBattery -= flyingTime;
            
            // 更新当前位置
            currentPos = bestReturnPoint;
            
            // 如果是车辆访问的任务点，需要等待车辆到达
            if (bestReturnPoint != drone.centerId && taskVisitInfo.count(bestReturnPoint) > 0) {
                double vehicleArrivalTime = taskVisitInfo.at(bestReturnPoint).second;
                // 更新到达时间（考虑等待车辆）
                currentTime = std::max(currentTime + flyingTime, vehicleArrivalTime);
            } else {
                // 直接返回配送中心，不需要等待
                currentTime += flyingTime;
            }
            
            // 在返回点充电和卸货
            currentBattery = drone.maxfuel;
            currentLoad = 0.0;
            maxProcessLoad = 0.0;
            
            // 记录到达时间
            times.push_back(currentTime);
        } else {
            // 无法找到返回点，规划失败
            return {{drone.centerId, drone.centerId}, {0.0, 0.0}};
        }
    }
    
    return {path, times};
}
