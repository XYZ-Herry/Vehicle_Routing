#include "task_assigner.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <cmath>
#include <unordered_map>

using std::vector;
using std::unordered_map;
using std::pair;
using std::make_pair;
using std::cout;
using std::endl;
using std::numeric_limits;

extern double droneMaxFuel;
extern double droneSpeed;
extern double vehicleSpeed;

// 将任务点分配给最近的配送中心
void assignTasksToCenters(DeliveryProblem& problem) {
    // 遍历所有静态任务，为每个任务找到最近的配送中心
    for (size_t i = 0; i < problem.initialDemandCount; i++) {
        
        auto& task = problem.tasks[i];
        double minCostTime = std::numeric_limits<double>::max();
        int closestCenterId = -1;
        
        for (const auto& center : problem.centers) {
            // 使用ID计算任务点到配送中心的实际距离（不是欧式距离）
            double distance = getDistance(task.id, center.id, problem, isDroneCenter(center));
            double cost_time = distance / (isDroneCenter(center) ? droneSpeed : vehicleSpeed);
            //如果该距离更短并且该中心如果是无人机配送中心且无人机可以抵达
            if (cost_time < minCostTime && (isCarCenter(center) || (isDroneCenter(center) && droneMaxFuel >= 2 * cost_time))) {
            //if (distance < minDistance) {
                minCostTime = cost_time;
                closestCenterId = center.id;
            }
        }
        
        if (closestCenterId != -1) {
            task.centerId = closestCenterId;  // 使用中心ID而非索引
        }
        problem.centerToTasks[closestCenterId].push_back(task.id);
    }
    

}
