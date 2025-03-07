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

// 将任务点分配给最近的配送中心
void assignTasksToCenters(DeliveryProblem& problem) {
    // 为每个配送中心创建任务列表
    vector<vector<int>> centerTasks(problem.centers.size());
    
    // 只处理初始需求点
    for (size_t i = 0; i < problem.initialDemandCount; ++i) {
        TaskPoint& task = problem.tasks[i];
        
        // 寻找距离此任务点最近的配送中心
        double minDist = std::numeric_limits<double>::max();
        int nearestCenter = -1;
        
        // 计算到每个配送中心的距离
        for (size_t j = 0; j < problem.centers.size(); ++j) {
            const DistributionCenter& center = problem.centers[j];
            double dist = sqrt(pow(task.x - center.x, 2) + pow(task.y - center.y, 2));
            
            bool canAssign = true;  // 默认可以分配
            
            // 如果是无人机中心，检查是否能够到达
            if (center.droneCount > 0) {
                // 找到该中心的一架无人机来检查速度
                double droneSpeed = 20.0;
                // for (const auto& vehicle : problem.vehicles) {
                //     if (vehicle.centerId == center.id && vehicle.maxLoad > 0) {
                //         droneSpeed = vehicle.speed;
                //         break;
                //     }
                // }
                
                if (droneSpeed > 0) {  // 确保找到了无人机
                    // 计算往返时间
                    //printf("%.2lf\n", dist);
                    double timeNeeded = 2 * dist / droneSpeed;
                    if (timeNeeded > DeliveryProblem::DEFAULT_DRONE_FUEL) {
                        canAssign = false;  // 无人机无法到达
                    }
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
            task.centerId = problem.centers[nearestCenter].id;  // 更新任务点的配送中心ID
        }
    }
    
    // 保存分配结果
    problem.centerAssignments = centerTasks;
    
    // 输出分配信息
    cout << "初始任务点分配到配送中心的结果:" << endl;
    for (int i = 0; i < problem.centers.size(); ++i) {
        cout << "配送中心 " << problem.centers[i].id << " (";
        if (isDroneCenter(problem.centers[i])) {
            cout << "无人机中心): ";
        } else {
            cout << "车辆中心): ";
        }
        cout << problem.centerAssignments[i].size() << " 个任务点" << endl;
    }
}
