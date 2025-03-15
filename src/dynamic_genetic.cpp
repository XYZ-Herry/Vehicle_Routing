#include "dynamic_genetic.h"
#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <limits>
#include <iostream>

using std::vector;
using std::pair;
using std::unordered_map;
using std::unordered_set;
using std::cout;
using std::endl;

// 动态阶段适应度计算 - 更新以使用新的数据结构
double calculateDynamicFitness(
    const vector<int>& solution,        // 存储车辆ID
    const vector<int>& allTaskIds,      // 存储任务ID
    const vector<TaskPoint>& tasks,
    const vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight,
    double staticMaxTime)
{
    // 使用映射检查车辆ID有效性
    for (size_t i = 0; i < allTaskIds.size(); ++i) {
        int vehicleId = solution[i];
        if (problem.vehicleIdToIndex.count(vehicleId) == 0) {
            return std::numeric_limits<double>::max();  // 无效的车辆ID
        }
    }
    
    // 转换为optimizeDynamicPaths所需的格式：vehicle-task对列表
    vector<pair<int, int>> assignments;
    for (size_t i = 0; i < allTaskIds.size(); ++i) {
        int vehicleId = solution[i];
        int taskId = allTaskIds[i];
        
        // 直接使用车辆ID
        assignments.push_back({vehicleId, taskId});  // (车辆ID, 任务ID)
    }
    
    // 使用optimizeDynamicPaths优化路径 - 现在返回map而不是vector
    auto optimizedPaths = optimizeDynamicPaths(problem, assignments, {});
    
    // 计算优化路径的适应度值
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    int tasksAssigned = 0;
    
    for (const auto& [vehicleId, pathData] : optimizedPaths) {
        const auto& [path, completionTimes] = pathData;
        if (path.size() <= 2) continue;  // 跳过没有任务的路径
        
        // 计算真实任务数量
        int realTaskCount = 0;
        for (size_t i = 0; i < path.size(); i++) {
            int pointId = path[i];
            // 直接使用problem.centerIds
            if (problem.centerIds.count(pointId) == 0) {
                // 如果不是配送中心ID，则是任务点
                realTaskCount++;
            }
        }
        
        // 记录任务数量
        tasksAssigned += realTaskCount;
        
        // 计算完成时间和成本
        if (!completionTimes.empty()) {
            maxCompletionTime = std::max(maxCompletionTime, completionTimes.back());
            
            // 查找车辆索引以获取成本
            int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
            totalCost += problem.vehicles[vehicleIndex].cost * realTaskCount;
        }
    }
    
    // 如果没有分配任务，返回无穷大
    if (tasksAssigned == 0) {
        return std::numeric_limits<double>::max();
    }
    
    // 考虑静态阶段的最大完成时间因素
    double dynamicTimePenalty = (maxCompletionTime > staticMaxTime) ? 
                               (maxCompletionTime - staticMaxTime) * 1.5 : 0.0;
    
    // 计算加权适应度值
    return timeWeight * (maxCompletionTime + dynamicTimePenalty) + 
           (1.0 - timeWeight) * totalCost;
}

// 改进的动态遗传算法，所有任务参与遗传 - 更新静态路径参数类型
vector<pair<int, int>> dynamicGeneticAlgorithm(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    const vector<int>& delayedTasks,
    const vector<int>& newTasks,
    int populationSize,
    int generations,
    double mutationRate,
    double timeWeight,
    double staticMaxTime)
{
    
    // 记录延迟和新增任务（可以自由分配）
    unordered_set<int> flexibleTasks;
    for (int taskId : delayedTasks) flexibleTasks.insert(taskId);
    for (int taskId : newTasks) flexibleTasks.insert(taskId);
    
    // 记录静态任务的原始分配信息
    struct TaskInfo {
        int centerId;    // 所属中心
        int vehicleId;   // 原始车辆ID（不是索引）
    };
    unordered_map<int, TaskInfo> staticTaskInfo;
    
    // 收集所有任务
    vector<int> allTaskIds;
    for (const auto& task : problem.tasks) {
        allTaskIds.push_back(task.id);
    }

    // 从静态路径中提取任务和分配信息
    for (const auto& [vehicleId, pathPair] : staticPaths) {
        const auto& path = pathPair.first;
        if (path.size() <= 2) continue;
        
        // 查找车辆索引
        int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            int taskId = path[i];
            
            // 如果不是延迟任务，记录其原始分配信息
            if (!flexibleTasks.count(taskId)) {
                staticTaskInfo[taskId] = {
                    problem.vehicles[vehicleIndex].centerId,
                    vehicleId  // 使用车辆ID
                };
            }
        }
    }
    
    // 初始化种群
    vector<vector<int>> population;
    int attempts = 0;
    const int maxAttempts = 1000;
    
    // 收集所有可用的车辆ID
    vector<int> allVehicleIds;
    for (const auto& vehicle : problem.vehicles) {
        allVehicleIds.push_back(vehicle.id);
    }
    
    // 尝试生成初始种群
    while (population.size() < populationSize && attempts < maxAttempts) {
        attempts++;
        vector<int> solution(allTaskIds.size());  // 存储车辆ID
        
        // 为每个任务分配车辆
        for (size_t i = 0; i < allTaskIds.size(); ++i) {
            int taskId = allTaskIds[i];
            
            if (flexibleTasks.count(taskId)) {
                // 延迟和新任务可以分配给任何车辆，使用随机车辆ID
                solution[i] = allVehicleIds[rand() % allVehicleIds.size()];
            } else {
                // 静态任务保持原有分配，使用车辆ID
                solution[i] = staticTaskInfo[taskId].vehicleId;
            }
        }
        
        // 验证解的可行性
        double fitness = calculateDynamicFitness(
            solution, allTaskIds, problem.tasks, 
            problem.vehicles, problem, timeWeight, staticMaxTime);
            
        if (fitness < std::numeric_limits<double>::max()) {
            population.push_back(solution);
        }
    }
    
    if (population.empty()) {
        cout << "无法生成有效的初始种群" << endl;
        return {};
    }
    
    // 遗传算法迭代
    for (int gen = 0; gen < generations; ++gen) {
        vector<pair<double, vector<int>>> fitnessPopulation;
        
        // 计算适应度
        for (const auto& solution : population) {
            double fitness = calculateDynamicFitness(
                solution, allTaskIds, problem.tasks, 
                problem.vehicles, problem, timeWeight, staticMaxTime);
            fitnessPopulation.push_back({fitness, solution});
        }
        
        sort(fitnessPopulation.begin(), fitnessPopulation.end());
        
        // 精英选择
        vector<vector<int>> newPopulation;
        int eliteCount = std::max(1, populationSize / 4);
        for (int i = 0; i < eliteCount && i < fitnessPopulation.size(); ++i) {
            newPopulation.push_back(fitnessPopulation[i].second);
        }
        
        // 交叉操作
        while (newPopulation.size() < populationSize && fitnessPopulation.size() >= 2) {
            // 选择父代
            int parent1Idx = rand() % std::min(populationSize/2, (int)fitnessPopulation.size());
            int parent2Idx = rand() % std::min(populationSize/2, (int)fitnessPopulation.size());
            
            vector<int> child1 = fitnessPopulation[parent1Idx].second;
            vector<int> child2 = fitnessPopulation[parent2Idx].second;
            
            // 单点交叉
            int crossPoint = rand() % allTaskIds.size();
            for (int j = 0; j < crossPoint; ++j) {
                std::swap(child1[j], child2[j]);
            }
            
            
            for (size_t i = 0; i < allTaskIds.size(); ++i) {
                int taskId = allTaskIds[i];
                if (!flexibleTasks.count(taskId)) {// 修正非超时和新加任务的分配
                    int centerId = staticTaskInfo[taskId].centerId;
                    
                    auto correctVehicle = [&](vector<int>& solution, int idx) {
                        int vehicleId = solution[idx];
                        if (problem.vehicles[vehicleId].centerId != centerId) {
                            // 使用centerIdToIndex直接查找中心
                            if (problem.centerIdToIndex.count(centerId)) {
                                int centerIdx = problem.centerIdToIndex.at(centerId);
                                const auto& center = problem.centers[centerIdx];
                                
                                if (!center.vehicles.empty()) {
                                    // 选择该中心的一辆车
                                    solution[idx] = center.vehicles[rand() % center.vehicles.size()];
                                }
                            }
                        }
                    };
                    
                    correctVehicle(child1, i);
                    correctVehicle(child2, i);
                }
            }
            
            // 验证并添加子代
            auto tryAddChild = [&](const vector<int>& child) {
                double fitness = calculateDynamicFitness(
                    child, allTaskIds, problem.tasks, 
                    problem.vehicles, problem, timeWeight, staticMaxTime);
                if (fitness < std::numeric_limits<double>::max()) {
                    newPopulation.push_back(child);
                }
            };
            
            tryAddChild(child1);
            if (newPopulation.size() < populationSize) {
                tryAddChild(child2);
            }
        }
        
        // 变异操作
        for (auto& solution : newPopulation) {
            if ((rand() % 100) < mutationRate * 100) {
                int taskIdx = rand() % allTaskIds.size();
                int taskId = allTaskIds[taskIdx];
                
                if (flexibleTasks.count(taskId)) {
                    // 只变异延迟和新任务
                    int oldVehicleId = solution[taskIdx];
                    solution[taskIdx] = allVehicleIds[rand() % allVehicleIds.size()];
                    
                    // 如果变异后不可行，恢复原值
                    if (calculateDynamicFitness(solution, allTaskIds, problem.tasks, 
                        problem.vehicles, problem, timeWeight, staticMaxTime) >= std::numeric_limits<double>::max()) {
                        solution[taskIdx] = oldVehicleId;
                    }
                }
            }
        }
        
        population = std::move(newPopulation);
    }
    
    // 返回最优解
    vector<pair<double, vector<int>>> finalPopulation;
    for (const auto& solution : population) {
        double fitness = calculateDynamicFitness(
            solution, allTaskIds, problem.tasks, 
            problem.vehicles, problem, timeWeight, staticMaxTime);
        finalPopulation.push_back({fitness, solution});
    }
    
    sort(finalPopulation.begin(), finalPopulation.end());
    
    // 构建最终分配结果
    vector<pair<int, int>> assignments;  // (车辆ID, 任务ID)对
    if (!finalPopulation.empty()) {
        const auto& bestSolution = finalPopulation[0].second;
        for (size_t i = 0; i < allTaskIds.size(); ++i) {
            int vehicleId = bestSolution[i];
            int taskId = allTaskIds[i];
            
            // 直接使用车辆ID
            assignments.push_back({vehicleId, taskId});
        }
    }
    
    return assignments;  // 返回(车辆ID, 任务ID)对
} 