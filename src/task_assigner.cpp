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
// 将任务点分配给最近的配送中心
void assignTasksToCenters(DeliveryProblem& problem) {
    // 遍历所有任务，为每个任务找到最近的配送中心
    for (size_t i = 0; i < problem.initialDemandCount; i++) {
        
        auto& task = problem.tasks[i];
        double minDistance = std::numeric_limits<double>::max();
        int closestCenterId = -1;
        
        for (const auto& center : problem.centers) {
            // 使用ID计算任务点到配送中心的实际距离
            double distance = getDistance(task.id, center.id, problem, false);
            if (distance < minDistance) {
                minDistance = distance;
                closestCenterId = center.id;
            }
        }
        
        if (closestCenterId != -1) {
            task.centerId = closestCenterId;  // 使用中心ID而非索引
        }
        problem.centerToTasks[closestCenterId].push_back(task.id);
    }
    

}
