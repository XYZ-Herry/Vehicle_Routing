#include "path_validator.h"
#include "path_optimizer.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

using std::vector;
using std::pair;
using std::string;
using std::cout;
using std::endl;

// 验证静态阶段路径中car/drone是否属于一开始分到的配送中心
bool validateStaticVehicleCenter(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths) 
{
    bool isValid = true;
    
    // 遍历所有路径
    for (const auto& [vehicleId, pathData] : staticPaths) {
        const auto& [path, times] = pathData;
        
        if (path.empty()) continue;
        
        // 获取Vehicle对象
        const Vehicle& vehicle = problem.vehicles[problem.vehicleIdToIndex.at(vehicleId)];
        
        // 检查car/drone是否属于其配送中心
        int vehicleCenterId = vehicle.centerId;
        int pathCenterId = path.front(); // 路径起点应该是配送中心
        
        if (vehicleCenterId != pathCenterId) {
            // 判断是car还是drone
            if (vehicle.maxLoad > 0) {
                cout << "错误: drone " << vehicleId << " 的路径不是从其所属配送中心 " 
                     << vehicleCenterId << " 出发，而是从 " << pathCenterId << " 出发" << endl;
            } else {
                cout << "错误: car " << vehicleId << " 的路径不是从其所属配送中心 " 
                     << vehicleCenterId << " 出发，而是从 " << pathCenterId << " 出发" << endl;
            }
            isValid = false;
        }
    }
    
    return isValid;
}

// 验证动态阶段中未超时的car/drone是否仍属于原配送中心
bool validateDynamicVehicleCenter(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths,
    double staticMaxTime)
{
    bool isValid = true;
    
    // 收集静态阶段中未超时的任务
    std::unordered_map<int, int> taskToVehicle; // 任务ID -> 分配的Vehicle ID
    
    for (const auto& [vehicleId, pathData] : staticPaths) {
        const auto& [path, times] = pathData;
        
        // 跳过空路径
        if (path.empty() || times.empty()) continue;
        
        // 获取Vehicle对象
        const Vehicle& vehicle = problem.vehicles[problem.vehicleIdToIndex.at(vehicleId)];
        
        // 遍历路径中的每个任务点
        for (size_t i = 0; i < path.size() - 1; ++i) { // 最后一个通常是返回配送中心
            int taskId = path[i];
            
            // 如果不是配送中心且未超时
            if (!problem.centerIds.count(taskId) && times[i] <= staticMaxTime) {
                taskToVehicle[taskId] = vehicleId;
            }
        }
    }
    
    // 验证动态阶段中未超时任务的分配是否一致
    for (const auto& [vehicleId, pathData] : dynamicPaths) {
        const auto& [path, times] = pathData;
        
        // 跳过空路径
        if (path.empty()) continue;
        
        // 遍历路径中的每个任务点
        for (size_t i = 0; i < path.size(); ++i) {
            int taskId = path[i];
            
            // 如果是静态阶段中未超时的任务
            if (taskToVehicle.count(taskId)) {
                int originalVehicleId = taskToVehicle[taskId];
                
                // 检查Vehicle是否属于同一个配送中心
                const Vehicle& originalVehicle = problem.vehicles[problem.vehicleIdToIndex.at(originalVehicleId)];
                const Vehicle& currentVehicle = problem.vehicles[problem.vehicleIdToIndex.at(vehicleId)];
                
                if (originalVehicle.centerId != currentVehicle.centerId) {
                    // 判断是car还是drone
                    if (currentVehicle.maxLoad > 0) {
                        cout << "错误: 动态阶段drone " << vehicleId << " 处理了任务 " << taskId 
                             << "，但该任务原本应由配送中心 " << originalVehicle.centerId 
                             << " 的Vehicle处理" << endl;
                    } else {
                        cout << "错误: 动态阶段car " << vehicleId << " 处理了任务 " << taskId 
                             << "，但该任务原本应由配送中心 " << originalVehicle.centerId 
                             << " 的Vehicle处理" << endl;
                    }
                    isValid = false;
                }
            }
        }
    }
    
    return isValid;
}

// 验证静态阶段路径合法性(drone电量、载重约束，时间计算)
std::pair<bool, std::string> validateStaticPathLegality(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths)
{
    bool isValid = true;
    string errorMessage = "";
    
    // 遍历所有路径
    for (const auto& [vehicleId, pathData] : staticPaths) {
        const auto& [path, reportedTimes] = pathData;
        
        // 跳过空路径
        if (path.empty() || path.size() < 2) continue;
        
        // 获取Vehicle对象
        const Vehicle& vehicle = problem.vehicles[problem.vehicleIdToIndex.at(vehicleId)];
        
        // 检查是否为drone（drone有最大载重和电量限制）
        bool isDrone = (vehicle.maxLoad > 0);
        
        // 自己计算时间以验证
        vector<double> calculatedTimes;
        calculatedTimes.push_back(0.0); // 起始时间为0
        
        if (isDrone) {
            double currentBattery = vehicle.maxfuel;
            double currentLoad = 0.0;
            double maxLoadDuringTrip = 0.0;
            int lastPointId = path[0]; // 从路径第一个点开始
            
            for (size_t i = 1; i < path.size(); ++i) { // 从第二个点开始，第一个点是起点
                int currentId = path[i];
                
                // 计算从上一点到当前点的距离和耗电量
                double distance = getDistance(lastPointId, currentId, problem, true);
                double batteryNeeded = distance / vehicle.speed;
                
                // 验证电量是否足够
                if (batteryNeeded > currentBattery) {
                    errorMessage += "错误: drone " + std::to_string(vehicleId) + 
                                   " 在前往任务点 " + std::to_string(currentId) + 
                                   " 时电量不足。需要: " + std::to_string(batteryNeeded) + 
                                   ", 剩余: " + std::to_string(currentBattery) + "\n";
                    isValid = false;
                }
                
                // 更新电量
                currentBattery -= batteryNeeded;
                
                // 计算到达时间
                double timeNeeded = batteryNeeded; // 对drone来说，消耗的时间等于消耗的电量
                double arrivalTime = calculatedTimes.back() + timeNeeded;
                calculatedTimes.push_back(arrivalTime);
                
                // 如果到达了配送中心，重置电量和载重
                if (problem.centerIds.count(currentId)) {
                    currentBattery = vehicle.maxfuel;
                    currentLoad = 0.0;
                    maxLoadDuringTrip = 0.0;
                } else {
                    // 非配送中心，则是任务点，更新载重
                    int taskIndex = problem.taskIdToIndex.at(currentId);
                    const TaskPoint& task = problem.tasks[taskIndex];
                    
                    // 更新载重
                    currentLoad += task.pickweight;
                    maxLoadDuringTrip = std::max(maxLoadDuringTrip, currentLoad - task.sendWeight);
                    
                    // 验证载重是否超过限制
                    if (currentLoad > vehicle.maxLoad || maxLoadDuringTrip > vehicle.maxLoad) {
                        errorMessage += "错误: drone " + std::to_string(vehicleId) + 
                                       " 在任务点 " + std::to_string(currentId) + 
                                       " 超过载重限制。当前载重: " + std::to_string(currentLoad) + 
                                       ", 最大载重: " + std::to_string(vehicle.maxLoad) + "\n";
                        isValid = false;
                    }
                }
                
                lastPointId = currentId;
            }
        } else {
            // 对于car，只需计算到达每个点的时间
            int lastPointId = path[0];
            
            for (size_t i = 1; i < path.size(); ++i) {
                int currentId = path[i];
                
                // 计算从上一点到当前点的距离和时间
                double distance = getDistance(lastPointId, currentId, problem, false);
                double timeNeeded = distance / vehicle.speed;
                
                // 计算到达时间
                double arrivalTime = calculatedTimes.back() + timeNeeded;
                calculatedTimes.push_back(arrivalTime);
                
                lastPointId = currentId;
            }
        }
        
        // 验证时间计算是否正确
        if (calculatedTimes.size() != reportedTimes.size()) {
            if (isDrone) {
                errorMessage += "错误: drone " + std::to_string(vehicleId) + 
                               " 的时间点数量不匹配。计算得到 " + 
                               std::to_string(calculatedTimes.size()) + 
                               ", 实际报告 " + std::to_string(reportedTimes.size()) + "\n";
            } else {
                errorMessage += "错误: car " + std::to_string(vehicleId) + 
                               " 的时间点数量不匹配。计算得到 " + 
                               std::to_string(calculatedTimes.size()) + 
                               ", 实际报告 " + std::to_string(reportedTimes.size()) + "\n";
            }
            isValid = false;
        } else {
            for (size_t i = 0; i < calculatedTimes.size(); ++i) {
                // 允许一定的浮点误差
                double timeDiff = std::abs(calculatedTimes[i] - reportedTimes[i]);
                if (timeDiff > 0.001) { // 允许误差0.001
                    if (isDrone) {
                        errorMessage += "错误: drone " + std::to_string(vehicleId) + 
                                       " 在点 " + std::to_string(path[i]) + 
                                       " 的时间计算不正确。计算得到 " + 
                                       std::to_string(calculatedTimes[i]) + 
                                       ", 实际报告 " + std::to_string(reportedTimes[i]) + "\n";
                    } else {
                        errorMessage += "错误: car " + std::to_string(vehicleId) + 
                                       " 在点 " + std::to_string(path[i]) + 
                                       " 的时间计算不正确。计算得到 " + 
                                       std::to_string(calculatedTimes[i]) + 
                                       ", 实际报告 " + std::to_string(reportedTimes[i]) + "\n";
                    }
                    isValid = false;
                }
            }
        }
    }
    
    return {isValid, errorMessage};
}

// 验证动态阶段路径合法性(考虑高峰期影响)
std::pair<bool, std::string> validateDynamicPathLegality(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths,
    const std::vector<int>& extraTaskIds)
{
    bool isValid = true;
    string errorMessage = "";
    
    // 创建额外任务ID的集合，便于快速查找
    std::unordered_set<int> extraTaskSet(extraTaskIds.begin(), extraTaskIds.end());
    
    // 首先收集所有Vehicle到达各点的时间，用于协同点验证
    std::unordered_map<int, double> vehicleArrivalTimes; // 任务点ID -> Vehicle到达时间
    
    // 阶段1：收集所有Vehicle的到达时间
    for (const auto& [vehicleId, pathData] : dynamicPaths) {
        const auto& [path, reportedTimes] = pathData;
        
        // 跳过空路径
        if (path.empty()) continue;
        
        // 获取Vehicle对象
        int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
        const Vehicle& vehicle = problem.vehicles[vehicleIndex];
        
        // 跳过drone
        if (vehicle.maxLoad > 0) continue;
        
        // 记录Vehicle到达每个点的时间
        for (size_t i = 0; i < path.size(); ++i) {
            int pointId = path[i];
            if (!problem.centerIds.count(pointId)) { // 如果不是配送中心
                vehicleArrivalTimes[pointId] = reportedTimes[i]; // 记录Vehicle到达时间
            }
        }
    }
    
    // 阶段2：验证所有路径
    for (const auto& [vehicleId, pathData] : dynamicPaths) {
        const auto& [path, reportedTimes] = pathData;
        
        // 跳过空路径
        if (path.empty()) continue;
        
        // 获取Vehicle对象
        int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
        const Vehicle& vehicle = problem.vehicles[vehicleIndex];
        
        // 是否为drone
        bool isDrone = vehicle.maxLoad > 0;
        
        // 手动计算时间，考虑高峰期和额外任务点约束
        vector<double> calculatedTimes;
        double currentTime = 0.0;  // 起点时间为0
        int lastPointId = path[0];  // 起点ID
        
        calculatedTimes.push_back(currentTime);  // 记录起点时间
        
        // 如果是drone，同时验证电量和载重约束
        double currentBattery = isDrone ? vehicle.maxfuel : 0.0;
        double currentLoad = 0.0;
        double maxLoadDuringTrip = 0.0;
        
        // 遍历路径上的每个点（从第二个点开始）
        for (size_t i = 1; i < path.size(); ++i) {
            int currentId = path[i];
            
            // 计算考虑高峰期的行驶时间
            double timeNeeded = calculateTimeNeeded(
                lastPointId, currentId, currentTime, 
                vehicle, problem, true, isDrone);
            
            // 到达当前点的时间（不考虑等待）
            double arrivalTime = currentTime + timeNeeded;
            
            // 检查是否为额外任务点且不是协同点，需要等待到指定时间
            if (extraTaskSet.count(currentId) && currentId < 30000) {
                int taskIndex = problem.taskIdToIndex.at(currentId);
                double requiredArrivalTime = problem.tasks[taskIndex].arrivaltime;
                
                // 如果到达时间早于要求时间，需要等待
                if (arrivalTime < requiredArrivalTime) {
                    arrivalTime = requiredArrivalTime;
                }
            }
            calculatedTimes.push_back(arrivalTime);//这里先记录到达当前时间，再更新
            // 检查是否为协同点，需要等待Vehicle到达
            bool isCollaborationPoint = (currentId >= 30000);
            if (isCollaborationPoint && isDrone) {
                int originalTaskId = currentId - 30000; // 获取原始任务点ID
                
                // 检查Vehicle是否访问了该点
                if (vehicleArrivalTimes.count(originalTaskId)) {
                    double vehicleArrivalTime = vehicleArrivalTimes[originalTaskId];
                    
                    // drone需要等待Vehicle到达
                    if (arrivalTime < vehicleArrivalTime) {
                        arrivalTime = vehicleArrivalTime;
                    }
                } else {
                    errorMessage += "错误: 动态阶段drone " + std::to_string(vehicleId) + 
                                  " 在协同点 " + std::to_string(currentId) + 
                                  " 等待的Vehicle未访问对应任务点 " + std::to_string(originalTaskId) + "\n";
                    isValid = false;
                }
            }
            
            // 更新当前时间
            currentTime = arrivalTime;
            
            // 对drone进行额外的电量和载重验证
            if (isDrone) {
                // 验证电量是否足够
                double batteryNeeded = timeNeeded;  // 对drone来说，耗电量等于飞行时间
                if (batteryNeeded > currentBattery) {
                    errorMessage += "错误: 动态阶段drone " + std::to_string(vehicleId) + 
                                   " 在前往任务点 " + std::to_string(currentId) + 
                                   " 时电量不足。需要: " + std::to_string(batteryNeeded) + 
                                   ", 剩余: " + std::to_string(currentBattery) + "\n";
                    isValid = false;
                }
                
                // 更新电量
                currentBattery -= batteryNeeded;
                
                // 如果到达了配送中心或车机协同点，重置电量和载重
                if (problem.centerIds.count(currentId) || currentId >= 30000) {
                    currentBattery = vehicle.maxfuel;
                    currentLoad = 0.0;
                    maxLoadDuringTrip = 0.0;
                } else {
                    // 非配送中心，则是任务点，更新载重
                    int taskIndex = problem.taskIdToIndex.at(currentId);
                    const TaskPoint& task = problem.tasks[taskIndex];
                    
                    // 更新载重
                    currentLoad += task.pickweight;
                    maxLoadDuringTrip = std::max(maxLoadDuringTrip, currentLoad - task.sendWeight);
                    
                    // 验证载重是否超过限制
                    if (currentLoad > vehicle.maxLoad || maxLoadDuringTrip > vehicle.maxLoad) {
                        errorMessage += "错误: 动态阶段drone " + std::to_string(vehicleId) + 
                                       " 在任务点 " + std::to_string(currentId) + 
                                       " 超过载重限制。当前载重: " + std::to_string(currentLoad) + 
                                       ", 最大载重: " + std::to_string(vehicle.maxLoad) + "\n";
                        isValid = false;
                    }
                }
            }
            
            lastPointId = currentId;
        }
        
        // 验证时间计算是否正确
        if (calculatedTimes.size() != reportedTimes.size()) {
            if (isDrone) {
                errorMessage += "错误: drone " + std::to_string(vehicleId) + 
                               " 的时间点数量不匹配。计算得到 " + 
                               std::to_string(calculatedTimes.size()) + 
                               ", 实际报告 " + std::to_string(reportedTimes.size()) + "\n";
            } else {
                errorMessage += "错误: car " + std::to_string(vehicleId) + 
                               " 的时间点数量不匹配。计算得到 " + 
                               std::to_string(calculatedTimes.size()) + 
                               ", 实际报告 " + std::to_string(reportedTimes.size()) + "\n";
            }
            isValid = false;
        } else {
            for (size_t i = 0; i < calculatedTimes.size(); ++i) {
                // 允许一定的浮点误差
                double timeDiff = std::abs(calculatedTimes[i] - reportedTimes[i]);
                if (timeDiff > 0.01) {  // 动态阶段允许稍大的误差，因为高峰期计算更复杂
                    if (isDrone) {
                        errorMessage += "错误: drone " + std::to_string(vehicleId) + 
                                       " 在点 " + std::to_string(path[i]) + 
                                       " 的时间计算不正确。计算得到 " + 
                                       std::to_string(calculatedTimes[i]) + 
                                       ", 实际报告 " + std::to_string(reportedTimes[i]) + "\n";
                    } else {
                        errorMessage += "错误: car " + std::to_string(vehicleId) + 
                                       " 在点 " + std::to_string(path[i]) + 
                                       " 的时间计算不正确。计算得到 " + 
                                       std::to_string(calculatedTimes[i]) + 
                                       ", 实际报告 " + std::to_string(reportedTimes[i]) + "\n";
                    }
                    isValid = false;
                }
            }
        }
    }
    
    return {isValid, errorMessage};
}

// 其他函数实现稍后添加... 