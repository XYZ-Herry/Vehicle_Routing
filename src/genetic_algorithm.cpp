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
double calculateFitness(
    const vector<int>& solution,
    const vector<int>& centerTasks,
    const vector<TaskPoint>& tasks,
    const vector<Vehicle>& vehicles,
    const DeliveryProblem& problem,
    double timeWeight)
{
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    unordered_map<int, vector<int>> vehicleAssignments;

    // 将任务分配给对应的车辆
    for (size_t i = 0; i < solution.size(); ++i) {
        int vehicleId = solution[i];
        int taskId = centerTasks[i];
        vehicleAssignments[vehicleId].push_back(taskId);
    }

    // 计算每辆车的路径和完成时间
    for (const auto& [vehicleId, assignments] : vehicleAssignments) {
        vector<int> path = optimizePathForVehicle(
            assignments, tasks, vehicles[vehicleId], problem);
        
        if (path.empty()) {
            return numeric_limits<double>::max();
        }
        
        vector<double> completionTimes = calculateCompletionTimes(
            path, tasks, vehicles[vehicleId], problem);
        
        if (!completionTimes.empty()) {
            maxCompletionTime = std::max(maxCompletionTime, completionTimes.back());
            
            // 计算成本：单位成本 * 任务数量
            totalCost += vehicles[vehicleId].cost * assignments.size();
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
        vector<int> centerTasks;      // 该中心负责的任务

        // 收集该中心的任务
        for (size_t i = 0; i < problem.tasks.size(); ++i) {
            if (problem.tasks[i].centerId == center.id) {
                centerTasks.push_back(i);
            }
        }
        // 直接使用 center.vehicles
        const vector<int>& centerVehicles = center.vehicles;

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
            double fitness = calculateFitness(solution, centerTasks, 
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
                double fitness = calculateFitness(individual, centerTasks, 
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
                int crossPoint = rand() % centerTasks.size();
                for (int j = 0; j <= crossPoint; ++j) {
                    std::swap(child1[j], child2[j]);
                }
                
                // 检查子代的可行性
                if (calculateFitness(child1, centerTasks, 
                    problem.tasks, problem.vehicles, problem, timeWeight) < std::numeric_limits<double>::max()) {
                    newPopulation.push_back(child1);
                }
                if (newPopulation.size() < populationSize && 
                    calculateFitness(child2, centerTasks, 
                        problem.tasks, problem.vehicles, problem, timeWeight) < std::numeric_limits<double>::max()) {
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
                    } while (calculateFitness(individual, centerTasks, 
                        problem.tasks, problem.vehicles, problem, timeWeight) >= std::numeric_limits<double>::max() &&
                            individual[taskIndex] != oldVehicle);
                    
                    if (calculateFitness(individual, centerTasks, 
                        problem.tasks, problem.vehicles, problem, timeWeight) >= std::numeric_limits<double>::max()) {
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