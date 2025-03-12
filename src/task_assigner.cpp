#include "task_assigner.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <cmath>

using std::vector;
using std::pair;
using std::make_pair;
using std::cout;
using std::endl;
using std::numeric_limits;
// 将任务点分配给最近的配送中心
void assignTasksToCenters(DeliveryProblem& problem) {
    // 获取第一个无人机的速度
    double droneSpeed = 0.0;
    for (const auto& vehicle : problem.vehicles) {
        if (vehicle.maxLoad > 0) {  // 是无人机
            droneSpeed = vehicle.speed;
            break;
        }
    }
    
    // 为每个配送中心创建任务列表
    vector<vector<int>> centerTasks(problem.centers.size());
    
    // 只处理初始需求点
    for (size_t i = 0; i < problem.initialDemandCount; ++i) {
        TaskPoint& task = problem.tasks[i];
        
        // 寻找距离此任务点最近的配送中心
        double minDist = numeric_limits<double>::max();
        int nearestCenter = -1;
        
        // 计算到每个配送中心的距离
        for (size_t j = 0; j < problem.centers.size(); ++j) {
            const DistributionCenter& center = problem.centers[j];
            double dist = getDistance(task.id, center.id, problem, 1);
            bool canAssign = true;  // 默认可以分配
            
            // 如果是无人机中心，检查是否能够到达
            if (isDroneCenter(center)) {
                double timeNeeded = 2 * dist / droneSpeed;  // 使用无人机的实际速度
                if (timeNeeded > DeliveryProblem::DEFAULT_DRONE_FUEL) {
                    canAssign = false;  // 无人机无法到达
                }
            }
            
            // 如果可以分配且距离更近，则更新最近中心
            if (canAssign && dist < minDist) {
                minDist = dist;
                nearestCenter = j;
            }
        }
        
        // 将任务分配给最近的配送中心
        if (nearestCenter >= 0) {
            centerTasks[nearestCenter].push_back(i);
            task.centerId = problem.centers[nearestCenter].id;
        }
    }
    
    // 将centerTasks转换为新的格式存储到problem.centerAssignments
    problem.centerAssignments.clear();
    for (size_t i = 0; i < problem.centers.size(); ++i) {
        int centerId = problem.centers[i].id;
        // 创建空的路径和时间向量
        vector<int> path;
        vector<double> times;
        if (!centerTasks[i].empty()) {
            // 如果有任务，添加到路径中
            path = centerTasks[i];
            times.resize(path.size(), 0.0);  // 初始化时间为0
        }
        problem.centerAssignments[centerId] = {path, times};
    }
    
    // 输出分配信息
    cout << "初始任务点分配到配送中心的结果:" << endl;
    for (size_t i = 0; i < problem.centers.size(); ++i) {
        cout << "配送中心 " << problem.centers[i].id << " (";
        if (isDroneCenter(problem.centers[i])) {
            cout << "无人机中心): ";
        } else {
            cout << "车辆中心): ";
        }
        cout << centerTasks[i].size() << " 个任务点" << endl;
    }
}
