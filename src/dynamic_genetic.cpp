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

// 动态阶段适应度计算
double calculateDynamicFitness(
    const vector<int>& solution,
    const vector<int>& allTasks,
    const vector<TaskPoint>& tasks,
    const vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight,
    double staticMaxTime)
{
    // 根据分配结果重组任务到车辆
    unordered_map<int, vector<int>> vehicleToTasks;
    for (size_t i = 0; i < allTasks.size(); ++i) {
        int taskId = allTasks[i];
        int vehicleId = solution[i];
        
        if (vehicleId < 0 || vehicleId >= vehicles.size()) {
            return std::numeric_limits<double>::max();  // 无效的车辆ID
        }
        
        vehicleToTasks[vehicleId].push_back(taskId);
    }
    
    // 优化每辆车的路径并计算指标
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    bool anyInfeasible = false;
    
    for (const auto& [vehicleId, vehicleTasks] : vehicleToTasks) {
        if (vehicleTasks.empty()) continue;
        
        // 为当前车辆优化路径
        vector<int> path = optimizePathForVehicle(
            vehicleTasks, tasks, vehicles[vehicleId], problem);
        
        if (path.empty()) {
            anyInfeasible = true;
            break;
        }
        
        // 计算考虑高峰期的完成时间
        vector<double> completionTimes = calculateCompletionTimes(
            path, tasks, vehicles[vehicleId], problem, true);
        
        if (!completionTimes.empty()) {
            maxCompletionTime = std::max(maxCompletionTime, completionTimes.back());
            totalCost += vehicles[vehicleId].cost * vehicleTasks.size();
        }
    }
    
    if (anyInfeasible) {
        return std::numeric_limits<double>::max();
    }
    
    // 考虑超过静态时间的惩罚
    double timePenalty = 0.0;
    if (maxCompletionTime > staticMaxTime) {
        timePenalty = (maxCompletionTime - staticMaxTime) * 2.0;
    }
    
    return timeWeight * (maxCompletionTime + timePenalty) + (1.0 - timeWeight) * totalCost;
}

// 改进的动态遗传算法，所有任务参与遗传
vector<pair<int, int>> dynamicGeneticAlgorithm(
    const DeliveryProblem& problem,
    const vector<pair<vector<int>, vector<double>>>& staticPaths,
    const vector<int>& delayedTasks,
    const vector<int>& newTasks,
    int populationSize,
    int generations,
    double mutationRate,
    double timeWeight,
    double staticMaxTime)
{
    // 提取静态阶段的所有任务分配 (taskId -> vehicleId)
    unordered_map<int, int> staticAssignments;
    for (size_t vehicleId = 0; vehicleId < staticPaths.size(); ++vehicleId) {
        const auto& path = staticPaths[vehicleId].first;
        if (path.size() < 2) continue;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            int taskId = path[i];
            staticAssignments[taskId] = vehicleId;
        }
    }
    
    // 统计所有需要参与遗传算法的任务（全部任务）
    vector<int> allTasks;
    for (const auto& [taskId, _] : staticAssignments) {
        allTasks.push_back(taskId);
    }
    
    // 添加新任务
    allTasks.insert(allTasks.end(), newTasks.begin(), newTasks.end());
    
    // 按配送中心组织车辆
    unordered_map<int, vector<int>> centerVehicles;
    for (size_t i = 0; i < problem.vehicles.size(); ++i) {
        centerVehicles[problem.vehicles[i].centerId].push_back(i);
    }
    
    // 记录任务是否为延迟或新增任务
    unordered_set<int> flexibleTasks;
    for (int taskId : delayedTasks) {
        flexibleTasks.insert(taskId);
    }
    for (int taskId : newTasks) {
        flexibleTasks.insert(taskId);
    }
    
    // 为静态任务记录所属中心
    unordered_map<int, int> taskCenters;
    for (const auto& [taskId, vehicleId] : staticAssignments) {
        if (flexibleTasks.count(taskId)) continue; // 跳过延迟/新增任务
        taskCenters[taskId] = problem.vehicles[vehicleId].centerId;
    }
    
    // 初始化种群
    vector<vector<int>> population;
    int attempts = 0;
    int maxAttempts = populationSize * 10;
    std::srand(std::time(nullptr));
    
    // 生成初始种群
    while (population.size() < populationSize && attempts < maxAttempts) {
        attempts++;
        
        vector<int> solution(allTasks.size());
        bool valid = true;
        
        // 为每个任务分配一个车辆
        for (size_t i = 0; i < allTasks.size(); ++i) {
            int taskId = allTasks[i];
            
            // 检查是否为延迟任务或新任务（可灵活分配）
            if (flexibleTasks.count(taskId)) {
                // 延迟任务和新任务可以分配给任何车辆
                solution[i] = rand() % problem.vehicles.size();
            }
            // 对于静态阶段已分配的任务
            else if (staticAssignments.count(taskId)) {
                // 在初始解中保持原有分配
                solution[i] = staticAssignments[taskId];
            }
        }
        
        // 验证解的可行性
        double fitness = calculateDynamicFitness(
            solution, allTasks, problem.tasks, 
            problem.vehicles, problem, timeWeight, staticMaxTime);
        
        if (fitness < std::numeric_limits<double>::max()) {
            population.push_back(solution);
        }
    }
    
    // 如果种群生成失败
    if (population.empty()) {
        cout << "无法生成有效的初始种群" << endl;
        return {};
    }
    
    // 开始迭代优化
    for (int gen = 0; gen < generations; ++gen) {
        // 计算适应度并排序
        vector<pair<double, vector<int>>> fitnessPopulation;
        for (const auto& solution : population) {
            double fitness = calculateDynamicFitness(
                solution, allTasks, problem.tasks, 
                problem.vehicles, problem, timeWeight, staticMaxTime);
            
            fitnessPopulation.push_back({fitness, solution});
        }
        
        std::sort(fitnessPopulation.begin(), fitnessPopulation.end());
        
        // 精英选择
        vector<vector<int>> newPopulation;
        int eliteCount = std::max(1, populationSize / 4);
        for (int i = 0; i < eliteCount && i < fitnessPopulation.size(); ++i) {
            newPopulation.push_back(fitnessPopulation[i].second);
        }
        
        // 交叉操作
        while (newPopulation.size() < populationSize && fitnessPopulation.size() >= 2) {
            int parent1Idx = rand() % std::min(populationSize/2, (int)fitnessPopulation.size());
            int parent2Idx;
            do {
                parent2Idx = rand() % std::min(populationSize/2, (int)fitnessPopulation.size());
            } while (parent1Idx == parent2Idx);
            
            vector<int> child1 = fitnessPopulation[parent1Idx].second;
            vector<int> child2 = fitnessPopulation[parent2Idx].second;
            
            // 单点交叉
            int crossPoint = rand() % allTasks.size();
            for (int j = 0; j < crossPoint; ++j) {
                std::swap(child1[j], child2[j]);
            }
            
            // 修正交叉结果
            for (size_t i = 0; i < allTasks.size(); ++i) {
                int taskId = allTasks[i];
                
                // 对于非延迟/非新增任务，检查并修正
                if (!flexibleTasks.count(taskId)) {
                    int originalCenterId = taskCenters[taskId];
                    
                    // 检查child1的分配是否在同一中心
                    int vehicle1 = child1[i];
                    int center1 = problem.vehicles[vehicle1].centerId;
                    if (center1 != originalCenterId) {
                        // 如果不在同一中心，随机选择原中心的一辆车
                        if (centerVehicles.count(originalCenterId) && !centerVehicles[originalCenterId].empty()) {
                            child1[i] = centerVehicles[originalCenterId][rand() % centerVehicles[originalCenterId].size()];
                        }
                    }
                    
                    // 同样检查child2
                    int vehicle2 = child2[i];
                    int center2 = problem.vehicles[vehicle2].centerId;
                    if (center2 != originalCenterId) {
                        if (centerVehicles.count(originalCenterId) && !centerVehicles[originalCenterId].empty()) {
                            child2[i] = centerVehicles[originalCenterId][rand() % centerVehicles[originalCenterId].size()];
                        }
                    }
                }
            }
            
            // 验证并添加到新种群
            double fitness1 = calculateDynamicFitness(
                child1, allTasks, problem.tasks, 
                problem.vehicles, problem, timeWeight, staticMaxTime);
            
            if (fitness1 < std::numeric_limits<double>::max()) {
                newPopulation.push_back(child1);
            }
            
            if (newPopulation.size() < populationSize) {
                double fitness2 = calculateDynamicFitness(
                    child2, allTasks, problem.tasks, 
                    problem.vehicles, problem, timeWeight, staticMaxTime);
                
                if (fitness2 < std::numeric_limits<double>::max()) {
                    newPopulation.push_back(child2);
                }
            }
        }
        
        // 变异操作
        for (auto& solution : newPopulation) {
            if ((rand() % 100) < mutationRate * 100) {
                int taskIdx = rand() % allTasks.size();
                int taskId = allTasks[taskIdx];
                int oldVehicle = solution[taskIdx];
                
                // 对于延迟任务和新任务，可以任意分配
                if (flexibleTasks.count(taskId)) {
                    solution[taskIdx] = rand() % problem.vehicles.size();
                }
                // 对于静态任务，只能在原中心内变异
                else {
                    int originalCenterId = taskCenters[taskId];
                    
                    if (centerVehicles.count(originalCenterId) && !centerVehicles[originalCenterId].empty()) {
                        solution[taskIdx] = centerVehicles[originalCenterId][rand() % centerVehicles[originalCenterId].size()];
                    }
                }
                
                // 验证变异结果
                double fitness = calculateDynamicFitness(
                    solution, allTasks, problem.tasks, 
                    problem.vehicles, problem, timeWeight, staticMaxTime);
                
                if (fitness >= std::numeric_limits<double>::max()) {
                    solution[taskIdx] = oldVehicle;  // 如果无效，恢复原值
                }
            }
        }
        
        // 更新种群
        population = std::move(newPopulation);
    }
    
    // 找出最佳解
    vector<pair<double, vector<int>>> finalPopulation;
    for (const auto& solution : population) {
        double fitness = calculateDynamicFitness(
            solution, allTasks, problem.tasks, 
            problem.vehicles, problem, timeWeight, staticMaxTime);
        finalPopulation.push_back({fitness, solution});
    }
    
    std::sort(finalPopulation.begin(), finalPopulation.end());
    
    // 转换为任务-车辆分配格式
    vector<pair<int, int>> assignments;
    if (!finalPopulation.empty()) {
        const auto& bestSolution = finalPopulation[0].second;
        
        for (size_t i = 0; i < allTasks.size(); ++i) {
            int vehicleId = bestSolution[i];
            int taskId = allTasks[i];
            
            // 添加所有任务的分配，不再筛选
            assignments.push_back({vehicleId, taskId});
        }
    }
    
    return assignments;
} 