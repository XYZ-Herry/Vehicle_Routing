#include "genetic_algorithm.h"
#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <unordered_map>
#include <random>
#include <cstdlib>

using std::vector;
using std::pair;
using std::make_pair;
using std::unordered_map;
using std::sort;

// 为指定配送中心随机生成初始解
vector<int> generateInitialSolution(const vector<int>& centerTasks, 
                                  const vector<int>& centerVehicleIndices)
{
    vector<int> solution(centerTasks.size());
    // 随机将任务分配给该中心的车辆
    for (size_t i = 0; i < centerTasks.size(); ++i) {
        int randomIndex = rand() % centerVehicleIndices.size();
        solution[i] = centerVehicleIndices[randomIndex];
    }
    return solution;
}

// 计算解的适应度（总成本 + 时间权重 * 完成时间）
double calculateFitness(const vector<int>& solution,
                       const vector<int>& centerTasks,
                       const vector<TaskPoint>& tasks,
                       const vector<Vehicle>& vehicles,
                       const RouteNetwork& network,
                       double timeWeight)
{
    double maxCompletionTime = 0.0;  // 最晚完成时间
    double totalCost = 0.0;          // 总运送成本
    unordered_map<int, vector<int>> vehicleAssignments;

    // 整理每辆车分配到的任务
    for (size_t i = 0; i < solution.size(); ++i) {
        vehicleAssignments[solution[i]].push_back(centerTasks[i]);
    }

    // 计算每辆车的运送成本和完成时间
    for (const auto& [vehicleId, assignedTasks] : vehicleAssignments) {
        if (!assignedTasks.empty()) {
            // 优化路径
            auto path = optimizePathForVehicle(assignedTasks, tasks, 
                                             vehicles[vehicleId], network);
            
            // 计算运送成本（只与货物数量有关）
            totalCost += vehicles[vehicleId].cost * assignedTasks.size();
            
            // 计算完成时间
            auto completionTimes = calculateCompletionTimes(path, tasks, 
                                                          vehicles[vehicleId], network);
            if (!completionTimes.empty()) {
                maxCompletionTime = std::max(maxCompletionTime, completionTimes.back());
            }
        }
    }

    // 返回加权目标函数值（较小的值更好）
    return (1.0 - timeWeight) * totalCost + timeWeight * maxCompletionTime;
}

// 遗传算法主函数
vector<pair<int, int>> geneticAlgorithm(
    const vector<TaskPoint>& tasks,
    const vector<Vehicle>& vehicles,
    const vector<DistributionCenter>& centers,
    int populationSize,
    int generations,
    double mutationRate,
    const RouteNetwork& network,
    double timeWeight)
{
    // 在函数内部初始化随机数种子
    static bool initialized = false;
    if (!initialized) {
        std::srand(std::time(nullptr));
        initialized = true;
    }
    
    vector<pair<int, int>> finalAssignments;    // 最终的任务分配方案 <vehicleId, taskId>
    
    // 对每个配送中心分别进行遗传算法优化
    for (const auto& center : centers) {
        vector<int> centerTasks;        // 该中心负责的任务ID列表
        vector<int> centerVehicles;     // 该中心拥有的车辆ID列表
        
        // 获取该中心的任务点
        for (size_t i = 0; i < tasks.size(); ++i) {
            if (tasks[i].centerId == center.id) {
                centerTasks.push_back(i);
            }
        }
        
        if (centerTasks.empty()) continue;

        // 获取该中心的车辆
        for (size_t i = 0; i < vehicles.size(); ++i) {
            if (vehicles[i].centerId == center.id) {
                centerVehicles.push_back(i);
            }
        }

        if (centerVehicles.empty()) continue;

        // 初始化种群
        vector<vector<int>> population;         // 种群，每个个体是一个任务分配方案
        vector<pair<double, vector<int>>> fitnessPopulation;  // <适应度值, 分配方案>
        
        for (int i = 0; i < populationSize; ++i) {
            population.push_back(generateInitialSolution(centerTasks, centerVehicles));
        }

        // 进行遗传算法迭代
        for (int gen = 0; gen < generations; ++gen) {
            fitnessPopulation.clear();

            // 计算种群中每个个体的适应度
            for (const auto& individual : population) {
                double fitness = calculateFitness(individual, centerTasks, tasks, 
                                                vehicles, network, timeWeight);
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
                int crossPoint = rand() % centerTasks.size();
                for (int j = 0; j <= crossPoint; ++j) {
                    std::swap(child1[j], child2[j]);
                }
                
                newPopulation.push_back(child1);
                if (newPopulation.size() < populationSize) {
                    newPopulation.push_back(child2);
                }
            }

            // 变异操作
            for (auto& individual : newPopulation) {
                if ((rand() % 100) < mutationRate * 100) {
                    int taskIndex = rand() % centerTasks.size();
                    int vehicleIndex = rand() % centerVehicles.size();
                    individual[taskIndex] = centerVehicles[vehicleIndex];
                }
            }

            population = newPopulation;

            // 在最后一代时，保存最优解的任务分配
            if (gen == generations - 1) {
                const auto& bestSolution = fitnessPopulation[0].second;
                for (size_t i = 0; i < centerTasks.size(); ++i) {
                    finalAssignments.push_back({bestSolution[i], centerTasks[i]});
                }
            }
        }
    }

    return finalAssignments;
}