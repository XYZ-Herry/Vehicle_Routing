#include "genetic_algorithm.h"
#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <unordered_map>
#include <random>
#include <cstdlib>
#include <limits>
#include <ctime>
#include <iostream>

using std::vector;
using std::pair;
using std::make_pair;
using std::unordered_map;
using std::numeric_limits;
using std::sort;
using std::cout;
using std::endl;

// 计算解的适应度
// solution: 车辆ID的数组，表示每个任务分配给哪个车辆
// centerTaskIds: 任务ID的数组，表示当前配送中心负责的所有任务ID
double calculateFitness(
    const vector<int>& solution,        // 存储车辆ID
    const vector<int>& centerTaskIds,   // 存储任务ID
    const vector<TaskPoint>& tasks,
    const vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight)
{
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    unordered_map<int, vector<int>> vehicleAssignments;  // 车辆ID -> 任务ID列表
    
    // 将任务分配给对应的车辆
    for (size_t i = 0; i < solution.size(); ++i) {
        int vehicleId = solution[i];     // 车辆ID
        int taskId = centerTaskIds[i];   // 任务ID
        vehicleAssignments[vehicleId].push_back(taskId);
    }
    
    // 对每个有任务的车辆计算路径和完成时间
    for (const auto& [vehicleId, taskIds] : vehicleAssignments) {
        // 获取车辆索引以访问车辆对象
        int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
        const Vehicle& vehicle = vehicles[vehicleIndex];
        
        // 优化该车辆的配送路径
        vector<int> path = optimizePathForVehicle(
            taskIds,  // 任务ID列表
            tasks, 
            vehicle,
            problem);
        
        if (path.empty()) {
            return numeric_limits<double>::max();
        }
        
        // 计算完成时间
        vector<double> completionTimes = calculateCompletionTimes(
            path,
            tasks, 
            vehicle, 
            problem);
        
        if (completionTimes.size() >= 2) {
            maxCompletionTime = std::max(maxCompletionTime, completionTimes[completionTimes.size()-2]);
        
            
            // 计算真实的任务数量(不包括配送中心)
            int taskCount = 0;
            for (size_t i = 0; i < path.size(); i++) {
                int pointId = path[i];
                // 直接使用problem.centerIds
                if (problem.centerIds.count(pointId) == 0) {
                    // 如果不是配送中心ID，则是任务点
                    taskCount++;
                }
            }
            
            totalCost += vehicle.cost * taskCount;
        }
    }
    
    return timeWeight * maxCompletionTime + (1.0 - timeWeight) * totalCost;
}

// 遗传算法主函数
vector<pair<int, int>> geneticAlgorithm(
    const DeliveryProblem& problem,
    int populationSize,
    int generations,
    double mutationRate,
    double timeWeight)
{
    vector<pair<int, int>> finalAssignments;
    srand(time(0));  // 随机数初始化

    // 按配送中心分组处理任务
    for (const auto& center : problem.centers) {
        vector<int> centerTaskIds;      // 该中心负责的任务ID
        
        // 收集该中心的任务ID
        for (size_t i = 0; i < problem.tasks.size(); ++i) {
            if (problem.tasks[i].centerId == center.id) {
                centerTaskIds.push_back(problem.tasks[i].id);  // 使用任务ID
            }
        }
        
        // 获取中心的车辆ID列表
        vector<int> centerVehicleIds;
        for (int vehicleId : center.vehicles) {
            centerVehicleIds.push_back(vehicleId);
        }

        if (centerTaskIds.empty() || centerVehicleIds.empty()) continue;

        // 初始化种群
        vector<vector<int>> population;
        int attempts = 0;
        const int maxAttempts = 1000;  // 最大尝试次数
        
        // 尝试生成初始种群
        while (population.size() < populationSize && attempts < maxAttempts) {
            vector<int> solution(centerTaskIds.size());  // solution里存的是车辆ID
            for (size_t i = 0; i < centerTaskIds.size(); ++i) {
                solution[i] = centerVehicleIds[rand() % centerVehicleIds.size()];
            }
            
            // 检查解的可行性
            double fitness = calculateFitness(solution, centerTaskIds, 
                problem.tasks, problem.vehicles, problem, timeWeight);
            if (fitness < std::numeric_limits<double>::max()) {
                population.push_back(solution);
            }
            attempts++;
        }

        // 如果无法找到足够的可行解，跳过这个配送中心
        if (population.empty()) {
            std::cout << "警告: 配送中心 #" << center.id << " 无法找到可行解，跳过处理" << std::endl;
            continue;
        }

        // 进行遗传算法迭代
        for (int gen = 0; gen < generations; ++gen) {
            vector<pair<double, vector<int>>> fitnessPopulation;

            // 计算种群中每个个体的适应度
            for (const auto& individual : population) {
                double fitness = calculateFitness(individual, centerTaskIds, 
                    problem.tasks, problem.vehicles, problem, timeWeight);
                fitnessPopulation.push_back({fitness, individual});
            }

            // 根据适应度排序（较小的适应度值更好）
            sort(fitnessPopulation.begin(), fitnessPopulation.end());
            
            // 生成新一代种群
            vector<vector<int>> newPopulation;
            
            // 精英选择：保留最优的一半个体
            for (int i = 0; i < populationSize / 2; ++i) {
                newPopulation.push_back(fitnessPopulation[i].second);
            }

            // 交叉操作
            while (newPopulation.size() < populationSize) {
                // 随机选择父代
                int parent1 = rand() % (populationSize / 2);
                int parent2 = rand() % (populationSize / 2);
                auto child1 = fitnessPopulation[parent1].second;
                auto child2 = fitnessPopulation[parent2].second;
                
                // 单点交叉
                int crossPoint = rand() % centerTaskIds.size();
                for (int j = 0; j <= crossPoint; ++j) {
                    std::swap(child1[j], child2[j]);
                }
                
                // 检查子代的可行性
                if (calculateFitness(child1, centerTaskIds, 
                    problem.tasks, problem.vehicles, problem, timeWeight) < std::numeric_limits<double>::max()) {
                    newPopulation.push_back(child1);
                }
                if (newPopulation.size() < populationSize && 
                    calculateFitness(child2, centerTaskIds, 
                        problem.tasks, problem.vehicles, problem, timeWeight) < std::numeric_limits<double>::max()) {
                    newPopulation.push_back(child2);
                }
            }

            // 变异操作
            for (auto& individual : newPopulation) {
                if ((rand() % 100) < mutationRate * 100) {
                    int taskIndex = rand() % centerTaskIds.size();
                    int oldVehicleId = individual[taskIndex];
                    do {
                        individual[taskIndex] = centerVehicleIds[rand() % centerVehicleIds.size()];
                    } while (calculateFitness(individual, centerTaskIds, 
                        problem.tasks, problem.vehicles, problem, timeWeight) >= std::numeric_limits<double>::max() &&
                            individual[taskIndex] != oldVehicleId);
                    
                    if (calculateFitness(individual, centerTaskIds, 
                        problem.tasks, problem.vehicles, problem, timeWeight) >= std::numeric_limits<double>::max()) {
                        individual[taskIndex] = oldVehicleId;  // 如果变异后不可行，恢复原值
                    }
                }
            }

            population = newPopulation;

            // 在最后一代时，保存最优解的任务分配
            if (gen == generations - 1) {
                // 计算最终种群的适应度
                fitnessPopulation.clear();
                for (const auto& individual : population) {
                    double fitness = calculateFitness(individual, centerTaskIds, 
                        problem.tasks, problem.vehicles, problem, timeWeight);
                    fitnessPopulation.push_back({fitness, individual});
                }
                sort(fitnessPopulation.begin(), fitnessPopulation.end());
                
                const auto& bestSolution = fitnessPopulation[0].second;
                for (size_t i = 0; i < centerTaskIds.size(); ++i) {
                    int vehicleId = bestSolution[i];
                    int taskId = centerTaskIds[i];
                    
                    // 直接使用车辆ID
                    finalAssignments.push_back({vehicleId, taskId});
                }
            }
        }
    }
    
    return finalAssignments;  // 返回(车辆ID, 任务ID)对
}