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
    // 检查解的基本有效性
    for (size_t i = 0; i < allTasks.size(); ++i) {
        int vehicleId = solution[i];
        if (vehicleId < 0 || vehicleId >= vehicles.size()) {
            return std::numeric_limits<double>::max();  // 无效的车辆ID
        }
    }
    
    // 转换为optimizeDynamicPaths所需的格式：vehicle-task对列表
    vector<pair<int, int>> assignments;
    for (size_t i = 0; i < allTasks.size(); ++i) {
        int vehicleId = solution[i];
        int taskId = allTasks[i];
        assignments.push_back({vehicleId, taskId});
    }
    
    // 使用optimizeDynamicPaths优化路径
    vector<pair<vector<int>, vector<double>>> optimizedPaths = 
        optimizeDynamicPaths(problem, assignments, {});
    
    // 计算优化路径的适应度值
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    int tasksAssigned = 0;
    
    for (size_t vehicleId = 0; vehicleId < optimizedPaths.size(); ++vehicleId) {
        const auto& [path, completionTimes] = optimizedPaths[vehicleId];
        
        // 跳过空路径或只有起点和终点的路径
        if (path.size() <= 2) continue;
        
        // 计算该车辆实际配送的任务数
        int vehicleTasks = path.size() - 2;  // 减去起点和终点
        tasksAssigned += vehicleTasks;
        
        // 更新最大完成时间
        if (!completionTimes.empty()) {
            maxCompletionTime = std::max(maxCompletionTime, completionTimes.back());
        }
        
        // 计算该车辆的成本
        totalCost += vehicles[vehicleId].cost * vehicleTasks;
    }
    
    // 检查是否所有任务都被成功分配
    if (tasksAssigned < allTasks.size()) {
        return std::numeric_limits<double>::max();  // 不是所有任务都能被分配，解无效
    }
    
    // 考虑超过静态时间的惩罚
    double timePenalty = 0.0;
    if (maxCompletionTime > staticMaxTime) {
        timePenalty = (maxCompletionTime - staticMaxTime) * 2.0;  // 惩罚因子
    }
    
    // 计算加权适应度
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
    // 收集所有配送中心ID以便过滤
    unordered_set<int> centerIds;
    for (const auto& center : problem.centers) {
        centerIds.insert(center.id);
    }
    
    // 记录延迟和新增任务（可以自由分配）
    unordered_set<int> flexibleTasks;
    for (int taskId : delayedTasks) flexibleTasks.insert(taskId);
    for (int taskId : newTasks) flexibleTasks.insert(taskId);
    
    // 记录静态任务的原始分配信息
    struct TaskInfo {
        int centerId;    // 所属中心
        int vehicleId;   // 原始车辆
    };
    unordered_map<int, TaskInfo> staticTaskInfo;
    
    // 收集所有任务
    vector<int> allTasks;
    
    // 从静态路径中提取任务和分配信息
    for (size_t vehicleId = 0; vehicleId < staticPaths.size(); ++vehicleId) {
        const auto& path = staticPaths[vehicleId].first;
        if (path.size() <= 2) continue;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            int taskId = path[i];
            
            // 确保不添加配送中心ID作为任务
            if (centerIds.count(taskId)) {
                continue;
            }
            
            allTasks.push_back(taskId);
            
            // 如果不是延迟任务，记录其原始分配信息
            if (!flexibleTasks.count(taskId)) {
                staticTaskInfo[taskId] = {
                    problem.vehicles[vehicleId].centerId,
                    (int)vehicleId
                };
            }
        }
    }
    
    // 添加新任务，确保不包含配送中心ID
    for (int taskId : newTasks) {
        if (!centerIds.count(taskId)) {
            allTasks.push_back(taskId);
        }
    }
    
    // 打印任务数量信息，方便调试
    cout << "动态优化任务数: " << allTasks.size() 
         << " (原始非延迟: " << (allTasks.size() - flexibleTasks.size()) 
         << ", 延迟或新增: " << flexibleTasks.size() << ")" << endl;
    
    // 初始化种群
    vector<vector<int>> population;
    int attempts = 0;
    int maxAttempts = populationSize * 10;
    
    // 生成初始种群
    while (population.size() < populationSize && attempts < maxAttempts) {
        attempts++;
        vector<int> solution(allTasks.size());
        
        // 为每个任务分配车辆
        for (size_t i = 0; i < allTasks.size(); ++i) {
            int taskId = allTasks[i];
            
            if (flexibleTasks.count(taskId)) {
                // 延迟和新任务可以分配给任何车辆
                solution[i] = rand() % problem.vehicles.size();
            } else {
                // 静态任务保持原有分配
                solution[i] = staticTaskInfo[taskId].vehicleId;
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
                solution, allTasks, problem.tasks, 
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
            int crossPoint = rand() % allTasks.size();
            for (int j = 0; j < crossPoint; ++j) {
                std::swap(child1[j], child2[j]);
            }
            
            
            for (size_t i = 0; i < allTasks.size(); ++i) {
                int taskId = allTasks[i];
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
                    child, allTasks, problem.tasks, 
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
                int taskIdx = rand() % allTasks.size();
                int taskId = allTasks[taskIdx];
                int oldVehicle = solution[taskIdx];
                
                if (flexibleTasks.count(taskId)) {
                    // 延迟和新任务可以分配给任何车辆
                    solution[taskIdx] = rand() % problem.vehicles.size();
                } else {
                    // 静态任务只能在原中心内变异
                    int centerId = staticTaskInfo[taskId].centerId;
                    
                    // 使用centerIdToIndex直接查找中心
                    if (problem.centerIdToIndex.count(centerId)) {
                        int centerIdx = problem.centerIdToIndex.at(centerId);
                        const auto& center = problem.centers[centerIdx];
                        
                        if (!center.vehicles.empty()) {
                            solution[taskIdx] = center.vehicles[rand() % center.vehicles.size()];
                        }
                    }
                }
                
                // 验证变异结果
                double fitness = calculateDynamicFitness(
                    solution, allTasks, problem.tasks, 
                    problem.vehicles, problem, timeWeight, staticMaxTime);
                    
                if (fitness >= std::numeric_limits<double>::max()) {
                    solution[taskIdx] = oldVehicle;  // 恢复无效的变异
                }
            }
        }
        
        population = std::move(newPopulation);
    }
    
    // 返回最优解
    vector<pair<double, vector<int>>> finalPopulation;
    for (const auto& solution : population) {
        double fitness = calculateDynamicFitness(
            solution, allTasks, problem.tasks, 
            problem.vehicles, problem, timeWeight, staticMaxTime);
        finalPopulation.push_back({fitness, solution});
    }
    
    sort(finalPopulation.begin(), finalPopulation.end());
    
    // 转换为任务-车辆分配格式
    vector<pair<int, int>> assignments;
    if (!finalPopulation.empty()) {
        const auto& bestSolution = finalPopulation[0].second;
        for (size_t i = 0; i < allTasks.size(); ++i) {
            assignments.push_back({bestSolution[i], allTasks[i]});
        }
    }
    
    return assignments;
} 