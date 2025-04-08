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

// 使用最近邻法优化静态阶段的配送路径
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
    
    // 检查是否为drone（drone有最大载重限制）
    bool isDrone = (vehicle.maxLoad > 0);
    
    if (!isDrone) {
        // 普通车辆使用原来的最近邻算法
        vector<int> path;
        vector<bool> visited(assignedTaskIds.size(), false);
        
        // 从配送中心开始
        path.push_back(centerId);
        int currentPos = centerId;
        
        const int MAX_ITERATIONS = 1000; // 设置合理的最大迭代次数
        int iteration = 0;
        
        // 根据距离选择下一个访问点，直到所有点都被访问
        while (anyTaskUnvisited(visited, assignedTaskIds) && iteration < MAX_ITERATIONS) {
            iteration++;
            
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
                        false); // 非drone
                    
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
        
        if (iteration >= MAX_ITERATIONS) {
            static int warningCount = 0;
            if (warningCount < 10) {
                std::cerr << "警告：静态阶段车辆路径优化达到最大迭代次数，ID: " << vehicle.id << std::endl;
                warningCount++;
                return {vehicle.centerId, vehicle.centerId};
            }
        }
        
        return path;
    } else {
        // drone路径规划，考虑电量和载重约束
        vector<int> path;
        vector<bool> visited(assignedTaskIds.size(), false);
        
        // 从配送中心开始
        path.push_back(centerId);
        int currentPos = centerId;
        
        // drone初始状态
        double currentBattery = vehicle.maxfuel; // 满电量
        double currentLoad = 0.0; // 初始载重为0
        double maxProcessLoad = 0.0; // 一次行程中的最大载重
        
        const int MAX_ITERATIONS = 1000; // 设置合理的最大迭代次数
        int iteration = 0;
        
        // 当还有未访问的任务点时继续循环
        while (anyTaskUnvisited(visited, assignedTaskIds) && iteration < MAX_ITERATIONS) {
            iteration++;
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
                    
                    // 添加10%最低电量约束 - 确保离开任务点后仍有至少10%的续航能力
                    double minRequiredBattery = vehicle.maxfuel * 0.1; // 10%的最大电量
                    if ((currentBattery - batteryNeededToTask) < minRequiredBattery) {
                        continue; // 剩余电量不足10%，跳过该任务点
                    }
                    
                    // 检查载重约束
                    if (task.sendWeight > 0) {
                        // 送货点：检查过程最大载重 + 送货重量是否超过最大载重
                        if (maxProcessLoad + task.sendWeight > vehicle.maxLoad) {
                            continue; // 超过载重限制，跳过该任务点
                        }
                    }
                    if (task.pickweight > 0) {
                        // 取货点：检查当前载重 + 取货重量是否超过最大载重
                        if (currentLoad + task.pickweight > vehicle.maxLoad) {
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
                
                if (nextTask.pickweight > 0) {
                    currentLoad += nextTask.pickweight;
                    maxProcessLoad = std::max(maxProcessLoad, currentLoad);
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
                    std::cerr << "警告: drone #" << vehicle.id << " 无解" << std::endl;
                    return {vehicle.centerId, vehicle.centerId}; // 返回空路径表示规划失败
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
                std::cerr << "警告: drone #" << vehicle.id << " 电量不足以返回配送中心！" << std::endl;
                return {vehicle.centerId, vehicle.centerId}; // 返回空路径表示规划失败
            }
        }
        
        if (iteration >= MAX_ITERATIONS) {
            static int warningCount = 0;
            if (warningCount < 10) {
                std::cerr << "警告：静态阶段无人机路径优化达到最大迭代次数，ID: " << vehicle.id << std::endl;
                warningCount++;
            }
            return {vehicle.centerId, vehicle.centerId}; // 返回空路径表示规划失败
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
    if (path.size() <= 2) {
        return {0.0, 0.0};
    }
    
    vector<double> completionTimes(path.size(), 0.0);
    double currentTime = 0.0;
    
    // 遍历路径中的每一段
    for (size_t i = 0; i < path.size() - 1; i++) {
        int fromId = path[i];
        int toId = path[i+1];
        
        // // 计算距离
        // double distance = getDistance(fromId, toId, problem, vehicle.maxLoad > 0);
        
        // // 如果考虑交通，车辆调整速度
        // double speed = vehicle.speed;
        // if (considerTraffic && vehicle.maxLoad > 0) {
        //     double speedFactor = getSpeedFactor(currentTime, fromId, toId, problem);
        //     speed *= speedFactor;
        // }
        
        // // 计算当前段的行驶时间
        // double travelTime = distance / speed;
        double travelTime = calculateTimeNeeded(fromId, toId, currentTime, vehicle, problem, considerTraffic, vehicle.maxLoad > 0);
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
        
        // 跳过drone
        if (vehicle.maxLoad > 0) continue;
        
        // 检查是否有分配的任务
        if (vehicleIdToTaskIds.count(vehicleId) && !vehicleIdToTaskIds[vehicleId].empty()) {
            // 使用新函数规划路径
            auto [path, times] = Dynamic_OptimizePathForVehicle(
                vehicleIdToTaskIds[vehicleId],
                problem.tasks,
                vehicle,
                problem
            );
            
            // 保存路径和时间
            dynamicPaths[vehicleId] = {path, times};
            
            // 记录任务点访问信息
            for (size_t i = 0; i < path.size(); ++i) {
                int pointId = path[i];
                if (problem.centerIds.count(pointId) == 0) {//不是配送中心
                    taskVisitInfo[pointId] = {vehicleId, times[i]};//记录任务点{车辆ID, 到达时间}
                }
            }
        } else {
            // 空路径
            dynamicPaths[vehicleId] = {{vehicle.centerId, vehicle.centerId}, {0.0, 0.0}};
        }
    }
    
    // 阶段2：规划drone路径，考虑与车辆协同
    for (const auto& drone : problem.vehicles) {
        int droneId = drone.id;
        
        // 只处理drone
        if (drone.maxLoad <= 0) continue;
        
        // 检查是否有分配的任务
        if (vehicleIdToTaskIds.count(droneId) && !vehicleIdToTaskIds[droneId].empty()) {
            // 使用协同算法规划drone路径
            auto [path, times] = optimizeDronePathWithVehicles(
                vehicleIdToTaskIds[droneId],
                problem.tasks,
                drone,
                problem,
                taskVisitInfo
            );
            
            if (path.size() > 2) {
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
                
                if (path.size() > 2) {
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

// 动态阶段为车辆优化路径，返回路径和时间
std::pair<std::vector<int>, std::vector<double>> Dynamic_OptimizePathForVehicle(
    const std::vector<int> &assignedTaskIds,
    const std::vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const DeliveryProblem& problem)
{
    if (assignedTaskIds.empty()) {
        return {{vehicle.centerId, vehicle.centerId}, {0.0, 0.0}};
    }
    
    std::vector<int> path;
    std::vector<double> times;
    std::vector<bool> visited(assignedTaskIds.size(), false);
    
    // 从配送中心开始
    int centerId = vehicle.centerId;
    path.push_back(centerId);
    times.push_back(0.0); // 初始时间
    
    int currentPos = centerId;
    double currentTime = 0.0;
    
    // 添加最大迭代次数限制
    int maxIterations = assignedTaskIds.size() * 3;
    int iterations = 0;

    
    // 根据距离选择下一个访问点，直到所有点都被访问
    while (anyTaskUnvisited(visited, assignedTaskIds) && iterations < maxIterations) {
        iterations++;
        
        double minDistance = std::numeric_limits<double>::max();
        int nextIndex = -1;
        int nextId = -1;
        
        // 寻找满足约束的下一个任务点
        for (size_t i = 0; i < assignedTaskIds.size(); i++) {
            if (!visited[i]) {
                int taskId = assignedTaskIds[i];
                int taskIndex = problem.taskIdToIndex.at(taskId);
                const TaskPoint& task = tasks[taskIndex];
                
                double distance = getDistance(currentPos, taskId, problem, false);
                
                // 考虑高峰期对速度的影响
                //double speedFactor = getSpeedFactor(currentTime, currentPos, taskId, problem);
                //double timeToTask = distance / (vehicle.speed * speedFactor);
                double timeToTask = calculateTimeNeeded(currentPos, taskId, currentTime, vehicle, problem, false, vehicle.maxLoad > 0);
                
                // 添加额外需求点到达时间约束
                if (taskIndex >= problem.initialDemandCount && 
                    currentTime + timeToTask + 0.000001 < task.arrivaltime) {
                    continue; // 额外需求点尚未到达，不能访问
                }
                
                if (distance < minDistance) {
                    minDistance = distance;
                    nextIndex = i;
                    nextId = taskId;
                }
            }
        }
        
        if (nextIndex != -1) {
            // 更新路径和状态
            visited[nextIndex] = true;
            path.push_back(nextId);
            
            // 考虑高峰期影响，计算实际行驶时间
            //double speedFactor = getSpeedFactor(currentTime, currentPos, nextId, problem);
            //double timeNeeded = minDistance / (vehicle.speed * speedFactor);
            double timeNeeded = calculateTimeNeeded(currentPos, nextId, currentTime, vehicle, problem, false, vehicle.maxLoad > 0);
            
            // 更新当前时间和位置
            currentTime += timeNeeded;
            currentPos = nextId;
            
            // 记录到达时间
            times.push_back(currentTime);
        }
        else{
            //找到最早的未完成的额外需求点
            int earliestExtraDemandId = -1;
            int earliestExtraDemandIndex = -1;
            double earliestArrivalTime = std::numeric_limits<double>::max();
            for (size_t i = 0; i < assignedTaskIds.size(); i++) {
                if (!visited[i]) {
                    int taskId = assignedTaskIds[i];
                    int taskIndex = problem.taskIdToIndex.at(taskId);
                    const TaskPoint& task = tasks[taskIndex];
                    
                    if (taskIndex >= problem.initialDemandCount && task.arrivaltime < earliestArrivalTime) {
                        earliestArrivalTime = task.arrivaltime;
                        earliestExtraDemandId = taskId;
                        earliestExtraDemandIndex = i;
                    }
                }
            }
            times.push_back(earliestArrivalTime);
            path.push_back(earliestExtraDemandId);
            visited[earliestExtraDemandIndex] = true;
            currentTime = earliestArrivalTime;
            currentPos = earliestExtraDemandId;
        }
    }
    
    // 返回配送中心
    if (currentPos != centerId) {
        path.push_back(centerId);
        
        //double distance = getDistance(currentPos, centerId, problem, false);
        //double speedFactor = getSpeedFactor(currentTime, currentPos, centerId, problem);
        //double timeNeeded = distance / (vehicle.speed * speedFactor);
        double timeNeeded = calculateTimeNeeded(currentPos, centerId, currentTime, vehicle, problem, false, vehicle.maxLoad > 0);
        
        currentTime += timeNeeded;
        times.push_back(currentTime);
    }
    
    // 记录未完成的任务
    if (iterations >= maxIterations) {
        static int warningCount = 0;
        if (warningCount < 10) {
            std::cerr << "警告: 动态阶段车辆路径规划达到最大迭代次数，可能存在死循环" << std::endl;
            warningCount++;
        }
        return {{vehicle.centerId, vehicle.centerId}, {0.0, 0.0}};
    }
    
    return {path, times};
}


// 考虑车辆协同的drone路径规划
std::pair<std::vector<int>, std::vector<double>> optimizeDronePathWithVehicles(
    const std::vector<int>& assignedTaskIds,
    const std::vector<TaskPoint>& tasks,
    const Vehicle& drone,
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<int, double>>& taskVisitInfo
) {
    if (assignedTaskIds.empty()) {
        return {{drone.centerId, drone.centerId}, {0.0, 0.0}};
    }
    
    std::vector<int> path;
    std::vector<double> times;  // 添加时间记录
    std::vector<bool> visited(assignedTaskIds.size(), false);
    int currentPos = drone.centerId;
    path.push_back(currentPos);
    times.push_back(0.0);  // 初始时间为0
    
    double currentBattery = drone.maxfuel;
    double currentLoad = 0.0;
    double maxProcessLoad = 0.0;
    double currentTime = 0.0;

    // 添加最大迭代次数限制
    int maxIterations = assignedTaskIds.size() * 3;
    int iterations = 0;
    bool backpoint_iscenter = true;
    
    // 当还有未访问的任务点时继续循环
    while (anyTaskUnvisited(visited, assignedTaskIds) && iterations < maxIterations) {
        iterations++;
        double minDistance = std::numeric_limits<double>::max();
        int nextIndex = -1;
        int nextId = -1;
        
        // 寻找满足约束的下一个任务点
        for (size_t i = 0; i < assignedTaskIds.size(); i++) {
            if (visited[i]) continue;
            
            int taskId = assignedTaskIds[i];
            int taskIndex = problem.taskIdToIndex.at(taskId);
            const TaskPoint& task = tasks[taskIndex];
            
            // 检查载重约束
            if (task.sendWeight > 0) {
                if (maxProcessLoad + task.sendWeight > drone.maxLoad) continue;
            }
            if (task.pickweight > 0) {
                if (currentLoad + task.pickweight > drone.maxLoad) continue;
            }
            
            // 计算到任务点的距离和电量需求
            double distanceToTask = getDistance(currentPos, taskId, problem, true);
            double batteryNeededToTask = distanceToTask / drone.speed;
            
            
            // 添加额外需求点到达时间约束
            if (taskIndex >= problem.initialDemandCount && currentTime + batteryNeededToTask + 0.000001 < task.arrivaltime) {
                //精度问题，给我坑惨了，调了一晚上 + 0.000001
                continue; // 额外需求点尚未到达，不能访问
            }
            
            // 检查电量是否足够到达任务点
            if (batteryNeededToTask > currentBattery) continue;
            
            
            // 检查从任务点是否有可行的返回点
            bool canReturn = false;
            
            // 首先检查是否可以返回原配送中心
            double distanceToOriginalCenter = getDistance(taskId, drone.centerId, problem, true);
            double batteryToOriginalCenter = distanceToOriginalCenter / drone.speed;
            
            // 添加10%最低电量约束 - 确保离开任务点后仍有至少10%的续航能力
            double minRequiredBattery = drone.maxfuel * 0.1; // 10%的最大电量
            double remainingBatteryAfterTask = currentBattery - batteryNeededToTask;
            
            
            // 如果剩余电量低于10%的门槛，跳过这个任务点
            if (remainingBatteryAfterTask < minRequiredBattery) continue;
            
            
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
                    
                    // 如果drone能到达该点，且在车辆到达前抵达
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
            backpoint_iscenter = false;
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
            
            if (task.pickweight > 0) {
                currentLoad += task.pickweight;
                maxProcessLoad = std::max(maxProcessLoad, currentLoad);
            }
            
            // 记录到达时间
            times.push_back(currentTime);
        } else {
            if (backpoint_iscenter){
                //找到最早的未完成的额外任务点
                int earliestExtraDemandId = -1;
                int earliestExtraDemandIndex = -1;
                double earliestArrivalTime = std::numeric_limits<double>::max();
                for (size_t i = 0; i < assignedTaskIds.size(); i++) {
                    if (!visited[i]) {
                        int taskId = assignedTaskIds[i];
                        int taskIndex = problem.taskIdToIndex.at(taskId);
                        if (taskIndex >= problem.initialDemandCount) {
                            if (tasks[taskIndex].arrivaltime < earliestArrivalTime) {
                                earliestArrivalTime = tasks[taskIndex].arrivaltime;
                                earliestExtraDemandId = taskId;
                                earliestExtraDemandIndex = i;
                            }
                        }
                    }
                }
                if (earliestExtraDemandId != -1) {
                    double distanceToEarliestDemand = getDistance(currentPos, earliestExtraDemandId, problem, true);
                    double timeToEarliestDemand = distanceToEarliestDemand / drone.speed;
                    const TaskPoint& task = tasks[earliestExtraDemandIndex];
                    
                    if (timeToEarliestDemand > currentBattery || currentLoad + task.pickweight > drone.maxLoad) {
                        // std::cout << "电量或载重约束不满足，跳过该任务点 " << earliestExtraDemandId 
                        //          << " (电量需求: " << timeToEarliestDemand << "/" << currentBattery 
                        //          << ", 载重需求: " << (currentLoad + task.pickweight) << "/" << drone.maxLoad << ")" << std::endl;
                        //visited[earliestExtraDemandIndex] = true;
                        continue;
                    }
                    
                    currentTime = earliestArrivalTime - timeToEarliestDemand;
                    //std::cout << "调整时间到 " << currentTime << " 以访问任务点 " << earliestExtraDemandId << std::endl;
                }
                else {
                    std::cout << "出错了，没点可以返回了" << std::endl;
                }
                continue;
            }
            backpoint_iscenter = true;
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
                
                // 如果drone能到达该点，且能在车辆到达前抵达，且最终返回时间更早
                if (batteryNeeded <= currentBattery && 
                    droneArrivalTime < vehicleArrivalTime && 
                    actualReturnTime < minReturnTime) {
                    minReturnTime = actualReturnTime;
                    bestReturnPoint = visitTaskId;
                }
            }
            
            // 如果找到可返回的点
            if (minReturnTime < std::numeric_limits<double>::max()) {
                if (bestReturnPoint != drone.centerId) path.push_back(bestReturnPoint + 30000);//协同点
                else path.push_back(bestReturnPoint);
                
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
                    times.push_back(currentTime + flyingTime);//这里先记录到达当前时间，再更新
                    currentTime = std::max(currentTime + flyingTime, vehicleArrivalTime);
                } else {
                    // 直接返回配送中心，不需要等待
                    currentTime += flyingTime;
                    times.push_back(currentTime);
                }
                
                // 在返回点充电和卸货
                currentBattery = drone.maxfuel;
                currentLoad = 0.0;
                maxProcessLoad = 0.0;
                
                // 记录到达时间
                //times.push_back(currentTime);
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
            
            // 如果drone能到达该点，且能在车辆到达前抵达，且最终返回时间更早
            if (batteryNeeded <= currentBattery && 
                droneArrivalTime < vehicleArrivalTime && 
                actualReturnTime < minReturnTime) {
                minReturnTime = actualReturnTime;
                bestReturnPoint = visitTaskId;
            }
        }
        
        // 如果找到可返回的点
        if (minReturnTime < std::numeric_limits<double>::max()) {
            //path.push_back(bestReturnPoint);
            if (bestReturnPoint != drone.centerId) path.push_back(bestReturnPoint + 30000);//协同点
            else path.push_back(bestReturnPoint);
            
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
                times.push_back(currentTime + flyingTime);
                currentTime = std::max(currentTime + flyingTime, vehicleArrivalTime);
            } else {
                // 直接返回配送中心，不需要等待
                currentTime += flyingTime;
                times.push_back(currentTime);
            }
            
            // 在返回点充电和卸货
            currentBattery = drone.maxfuel;
            currentLoad = 0.0;
            maxProcessLoad = 0.0;
            
            // 记录到达时间
            //times.push_back(currentTime);
        } else {
            // 无法找到返回点，规划失败
            return {{drone.centerId, drone.centerId}, {0.0, 0.0}};
        }
    }
    
    // 记录未完成的任务
    if (iterations >= maxIterations) {
        static int warningCount = 0;
        if (warningCount < 10) {
            std::cerr << "警告: 动态阶段车机协同路径规划达到最大迭代次数，可能存在死循环" << std::endl;
            warningCount++;
        }
        return {{drone.centerId, drone.centerId}, {0.0, 0.0}};
    }
    
    return {path, times};
}

// 计算从一个点到另一个点需要的时间
double calculateTimeNeeded(
    int currentId,      // 当前点id
    int destId,         // 目的点id
    double currentTime, // 当前时间
    const Vehicle& vehicle,
    const DeliveryProblem& problem,
    bool considerTraffic,  // 是否考虑高峰期
    bool isDrone)  // 是否是drone
{
    // 获取两点之间的距离
    double distance = getDistance(currentId, destId, problem, isDrone);
    
    // 如果不考虑高峰期或者是drone，直接计算
    if (!considerTraffic || isDrone) {
        return distance / vehicle.speed;
    }
    
    // 车辆且考虑高峰期的情况，需要分段计算
    double remainingDistance = distance;
    double totalTime = 0.0;
    double travelTime = currentTime;
    
    // 高峰期时间段定义
    const double morningPeakStart = DeliveryProblem::MORNING_PEAK_START;
    const double morningPeakEnd = DeliveryProblem::MORNING_PEAK_END;
    const double eveningPeakStart = DeliveryProblem::EVENING_PEAK_START;
    const double eveningPeakEnd = DeliveryProblem::EVENING_PEAK_END;
    
    // 车辆在正常时段的速度
    double normalSpeed = vehicle.speed;
    
    // 持续计算直到所有距离都已经行驶
    while (remainingDistance > 0.0001) {
        // 判断当前时刻是否在高峰期
        bool isMorningPeak = (travelTime >= morningPeakStart && travelTime < morningPeakEnd);
        bool isEveningPeak = (travelTime >= eveningPeakStart && travelTime < eveningPeakEnd);
        bool isPeakHour = isMorningPeak || isEveningPeak;
        
        // 当前速度系数和速度
        double speedFactor = 1.0;
        if (isPeakHour) {
            speedFactor = getSpeedFactor(travelTime, currentId, destId, problem);
        }
        double currentSpeed = normalSpeed * speedFactor;
        
        // 计算到下一个时间段的时间
        double timeToNextPhase = std::numeric_limits<double>::max();
        
        if (travelTime < morningPeakStart) {
            // 当前在早高峰前
            timeToNextPhase = morningPeakStart - travelTime;
        } else if (travelTime < morningPeakEnd) {
            // 当前在早高峰中
            timeToNextPhase = morningPeakEnd - travelTime;
        } else if (travelTime < eveningPeakStart) {
            // 当前在早高峰后，晚高峰前
            timeToNextPhase = eveningPeakStart - travelTime;
        } else if (travelTime < eveningPeakEnd) {
            // 当前在晚高峰中
            timeToNextPhase = eveningPeakEnd - travelTime;
        } else {
            // 当前在晚高峰后
            timeToNextPhase = 24.0 - travelTime + morningPeakStart; // 到第二天早高峰开始的时间
        }
        
        // 以当前速度能行驶的距离
        double distanceCanTravel = currentSpeed * timeToNextPhase;
        
        if (distanceCanTravel >= remainingDistance) {
            // 如果可以到达目的地
            double segmentTime = remainingDistance / currentSpeed;
            totalTime += segmentTime;
            remainingDistance = 0;
        } else {
            // 如果不能到达目的地，行驶至下一个时间段
            totalTime += timeToNextPhase;
            remainingDistance -= distanceCanTravel;
            travelTime += timeToNextPhase;
            
            // 处理一天结束的情况
            if (travelTime >= 24.0) {
                travelTime -= 24.0;
            }
        }
    }
    
    return totalTime;
}
