#include "static_genetic.h"
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
// solution: 车辆ID的数组，表示每个任务分配给哪个车辆， solution[i]的i表示centerTaskIds的下标
// centerTaskIds: 任务ID的数组，表示当前配送中心负责的所有任务ID
double calculateFitness(
    const vector<int>& solution,        // 存储车辆ID
    const vector<int>& centerTaskIds,   // 存储任务ID
    const vector<TaskPoint>& tasks,
    const vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight)
{
    static int callCount = 0;
    callCount++;

    // 每1000次调用输出一次状态
    if (callCount % 1000 == 0) {
        std::cout << "静态阶段适应度计算次数: " << callCount << std::endl;
    }
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
        
        if (path.size() <= 2) {
            return numeric_limits<double>::max();
        }
        
        // 计算完成时间
        vector<double> completionTimes = calculateCompletionTimes(
            path,
            tasks, 
            vehicle, 
            problem,
            false);
        
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
vector<pair<int, int>> Static_GeneticAlgorithm(
    const DeliveryProblem& problem,
    int populationSize,
    int generations,
    double mutationRate,
    double timeWeight)
{
    vector<pair<int, int>> finalAssignments;
    srand(time(nullptr));  // 随机数初始化

    // 按配送中心分组处理任务
    for (const auto& center : problem.centers) {
        // 直接从problem.centerToTasks获取该中心负责的任务ID
        auto tasksIt = problem.centerToTasks.find(center.id);
        if (tasksIt == problem.centerToTasks.end() || tasksIt->second.empty()) {
            cout << "警告: 配送中心 #" << center.id << " 没有任务，跳过处理" << endl;
            continue;  // 跳过没有任务的中心
        }
        
        const vector<int>& centerTaskIds = tasksIt->second;  // 该中心负责的任务ID
        
        // 获取中心的车辆ID列表（已经存储在center.vehicles中）
        vector<int> centerVehicleIds = center.vehicles;

        if (centerVehicleIds.empty()) {
            cout << "警告: 配送中心 #" << center.id << " 没有车辆，跳过处理" << endl;
            continue;
        }

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
            //std::cout << "尝试次数: " << attempts << std::endl;
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
                    
                    // 尝试最多10次找到可行的变异
                    bool foundValid = false;
                    for (int attempt = 0; attempt < 10; ++attempt) {
                        // 选择新的车辆ID
                        int newVehicleId = centerVehicleIds[rand() % centerVehicleIds.size()];
                        if (newVehicleId == oldVehicleId) continue; // 跳过相同的车辆
                        
                        // 临时应用变异
                        individual[taskIndex] = newVehicleId;
                        
                        // 计算一次适应度
                        double fitness = calculateFitness(individual, centerTaskIds, 
                            problem.tasks, problem.vehicles, problem, timeWeight);
                        
                        // 检查可行性
                        if (fitness < std::numeric_limits<double>::max()) {
                            foundValid = true;
                            break; // 找到可行解，退出循环
                        }
                        
                        // 不可行，恢复原值
                        individual[taskIndex] = oldVehicleId;
                    }
                    
                    // 如果没有找到可行解，确保还原到原始状态
                    if (!foundValid) {
                        individual[taskIndex] = oldVehicleId;
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