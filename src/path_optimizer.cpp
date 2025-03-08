#include "path_optimizer.h"
#include "common.h"
#include <algorithm>
#include <limits>
#include <cmath>

using std::vector;
using std::numeric_limits;

// 使用最近邻法优化车辆的配送路径
vector<int>  optimizePathForVehicle(
    const vector<int> &assignedTasks, 
    const vector<TaskPoint> &tasks, 
    const Vehicle &vehicle,
    const DeliveryProblem& problem)
{
    if (assignedTasks.empty()) {
        return {vehicle.centerId};
    }

    vector<int> path;
    vector<bool> visited(tasks.size());
    int centerId = vehicle.centerId;
    double currentLoad = 0.0;
    double remainingFuel = vehicle.fuel;
    int currentPos = centerId;
    
    path.push_back(centerId);
    
    while (true) {
        double minDist = numeric_limits<double>::max();
        int nextTask = -1;
        bool allTasksNeedReturn = true;

        for (int taskIndex : assignedTasks) {
            if (!visited[taskIndex]) {
                // 计算到达该点的距离和时间
                double dist = getDistance(
                    currentPos == centerId ? centerId : tasks[currentPos].id,
                    tasks[taskIndex].id,
                    problem,
                    vehicle.maxLoad > 0
                );
                
                double timeNeeded = dist / vehicle.speed;
                bool canVisitDirectly = true;
                
                if (vehicle.maxLoad > 0) {
                    if (currentLoad + tasks[taskIndex].weight > vehicle.maxLoad) {
                        canVisitDirectly = false;
                    }

                    if (timeNeeded > remainingFuel) {
                        canVisitDirectly = false;
                    }
                    
                    // 计算返回配送中心的距离
                    double returnDist = getDistance(
                        tasks[taskIndex].id,
                        centerId,
                        problem,
                        true
                    );
                    double returnTime = returnDist / vehicle.speed;
                    
                    if (timeNeeded + returnTime > remainingFuel) {
                        canVisitDirectly = false;
                    }
                }
                
                if (canVisitDirectly) {
                    allTasksNeedReturn = false;
                    if (dist < minDist) {
                        minDist = dist;
                        nextTask = taskIndex;
                    }
                }
            }
        }

        if (allTasksNeedReturn && anyTaskUnvisited(visited, assignedTasks)) {
            path.push_back(centerId);
            currentLoad = 0.0;
            remainingFuel = vehicle.fuel;
            currentPos = centerId;
            continue;
        }

        if (nextTask != -1) {
            visited[nextTask] = true;
            path.push_back(nextTask);
            
            if (vehicle.maxLoad > 0) {
                currentLoad += tasks[nextTask].weight;
                remainingFuel -= minDist / vehicle.speed;
            }
            currentPos = nextTask;
        } else {
            break;
        }
    }

    bool allVisited = true;
    for (int taskIndex : assignedTasks) {
        if (!visited[taskIndex]) {
            allVisited = false;
            break;
        }
    }
    
    if (!allVisited) {
        return vector<int>();
    }

    if (currentPos != centerId) {
        path.push_back(centerId);
    }
    
    return path;
}

// 辅助函数：检查是否还有未访问的任务点
bool anyTaskUnvisited(const vector<bool>& visited, const vector<int>& tasks) {
    for (int taskIndex : tasks) {
        if (!visited[taskIndex]) {
            return true;
        }
    }
    return false;
}

// 计算某辆车路径上每个任务点的完成时间
vector<double> calculateCompletionTimes(
    const vector<int> &path, 
    const vector<TaskPoint> &tasks,
    const Vehicle &vehicle,
    const DeliveryProblem& problem)
{
    if (path.empty()) {
        return {0.0};
    }
    
    vector<double> completionTimes;
    double currentTime = 0.0;
    double currentLoad = 0.0;
    int currentPos = vehicle.centerId;
    
    for (size_t i = 1; i < path.size(); ++i) {
        int nextPos = path[i];
        
        // 计算从当前位置到下一位置的距离和时间
        double dist = getDistance(
            currentPos == vehicle.centerId ? vehicle.centerId : tasks[currentPos].id,
            nextPos == vehicle.centerId ? vehicle.centerId : tasks[nextPos].id,
            problem,
            vehicle.maxLoad > 0
        );
        
        currentTime += dist / vehicle.speed;
        
        if (nextPos == vehicle.centerId) {
            currentLoad = 0.0;
        } else {
            currentLoad += tasks[nextPos].weight;
            completionTimes.push_back(currentTime);
        }
        
        currentPos = nextPos;
    }
    
    if (completionTimes.empty()) {
        completionTimes.push_back(0.0);
    }
    
    return completionTimes;
}