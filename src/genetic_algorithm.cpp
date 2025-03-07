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
using std::sort;
using std::cout;
using std::endl;

// 计算解的适应度（总成本 + 时间权重 * 完成时间）
double calculateFitness(
    const std::vector<int>& solution,
    const std::vector<int>& centerTasks,
    const std::vector<TaskPoint>& tasks,
    const std::vector<Vehicle>& vehicles,
    const RouteNetwork& network,
    double timeWeight,
    const std::vector<DistributionCenter>& centers)
{
    double maxCompletionTime = 0.0;  // 最晚完成时间
    double totalCost = 0.0;          // 总运送成本
    unordered_map<int, vector<int>> vehicleAssignments;

    // 将任务分配给对应的车辆
    for (size_t i = 0; i < solution.size(); ++i) {
        int vehicleId = solution[i];
        int taskId = centerTasks[i];
        vehicleAssignments[vehicleId].push_back(taskId);
    }

    // 计算每辆车的路径和完成时间
    for (const auto& [vehicleId, assignments] : vehicleAssignments) {
        // 使用最近邻法优化路径
        vector<int> path = optimizePathForVehicle(
            assignments, tasks, vehicles[vehicleId], network, centers);
        
        // 如果返回空路径，说明解不可行
        if (path.empty()) {
            return std::numeric_limits<double>::max();  // 返回一个很大的惩罚值
        }
        
        // 计算完成时间
        vector<double> completionTimes = calculateCompletionTimes(
            path, tasks, vehicles[vehicleId], network, centers);
            
        if (!completionTimes.empty()) {
            double vehicleTime = completionTimes.back();
            maxCompletionTime = std::max(maxCompletionTime, vehicleTime);
            double distance = 0.0;
            for (size_t i = 0; i < path.size() - 1; ++i) {
                int p1 = path[i];
                int p2 = path[i + 1];
                if (p1 == vehicles[vehicleId].centerId || p2 == vehicles[vehicleId].centerId) {
                    // 如果涉及配送中心，需要特殊处理距离计算
                    double centerX = 0, centerY = 0;
                    for (const auto& center : centers) {
                        if (center.id == vehicles[vehicleId].centerId) {
                            centerX = center.x;
                            centerY = center.y;
                            break;
                        }
                    }
                    if (p1 == vehicles[vehicleId].centerId) {
                        distance += sqrt(pow(tasks[p2].x - centerX, 2) + pow(tasks[p2].y - centerY, 2));
                    } else {
                        distance += sqrt(pow(tasks[p1].x - centerX, 2) + pow(tasks[p1].y - centerY, 2));
                    }
                } else {
                    distance += getDistance(tasks[p1], tasks[p2], network, vehicles[vehicleId].maxLoad > 0);
                }
            }
            totalCost += distance * vehicles[vehicleId].cost;
        }
    }

    return timeWeight * maxCompletionTime + (1.0 - timeWeight) * totalCost;
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
    vector<pair<int, int>> finalAssignments;
    srand(time(0));  // 随机数初始化

    // 按配送中心分组处理任务
    for (const auto& center : centers) {
        vector<int> centerTasks;      // 该中心负责的任务
        vector<int> centerVehicles;   // 该中心的车辆

        // 收集该中心的任务和车辆
        for (size_t i = 0; i < tasks.size(); ++i) {
            if (tasks[i].centerId == center.id) {
                centerTasks.push_back(i);
            }
        }
        for (size_t i = 0; i < vehicles.size(); ++i) {
            if (vehicles[i].centerId == center.id) {
                centerVehicles.push_back(i);
            }
        }

        if (centerTasks.empty() || centerVehicles.empty()) continue;

        // 初始化种群
        vector<vector<int>> population;
        int attempts = 0;
        const int maxAttempts = 1000;  // 最大尝试次数
        
        // 尝试生成初始种群
        while (population.size() < populationSize && attempts < maxAttempts) {
            vector<int> solution(centerTasks.size());
            for (size_t i = 0; i < centerTasks.size(); ++i) {
                solution[i] = centerVehicles[rand() % centerVehicles.size()];
            }
            
            // 检查解的可行性
            double fitness = calculateFitness(solution, centerTasks, tasks, vehicles, network, timeWeight, centers);
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
                double fitness = calculateFitness(individual, centerTasks, tasks, vehicles, network, timeWeight, centers);
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
                
                // 检查子代的可行性
                if (calculateFitness(child1, centerTasks, tasks, vehicles, network, timeWeight, centers) < std::numeric_limits<double>::max()) {
                    newPopulation.push_back(child1);
                }
                if (newPopulation.size() < populationSize && 
                    calculateFitness(child2, centerTasks, tasks, vehicles, network, timeWeight, centers) < std::numeric_limits<double>::max()) {
                    newPopulation.push_back(child2);
                }
            }

            // 变异操作
            for (auto& individual : newPopulation) {
                if ((rand() % 100) < mutationRate * 100) {
                    int taskIndex = rand() % centerTasks.size();
                    int oldVehicle = individual[taskIndex];
                    do {
                        individual[taskIndex] = centerVehicles[rand() % centerVehicles.size()];
                    } while (calculateFitness(individual, centerTasks, tasks, vehicles, network, timeWeight, centers) >= std::numeric_limits<double>::max() &&
                            individual[taskIndex] != oldVehicle);
                    
                    if (calculateFitness(individual, centerTasks, tasks, vehicles, network, timeWeight, centers) >= std::numeric_limits<double>::max()) {
                        individual[taskIndex] = oldVehicle;  // 如果变异后不可行，恢复原值
                    }
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