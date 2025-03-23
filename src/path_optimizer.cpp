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
    
    // 使用贪心算法（最近邻法）构建路径
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
                
                // 直接使用任务ID计算距离
                double distance = getDistance(
                    currentPos, 
                    taskId, 
                    problem,
                    vehicle.maxLoad > 0);
                
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
    
    if (currentPos != centerId) {
        path.push_back(centerId);
    }
    
    return path;
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
