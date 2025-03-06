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
            // 计算欧几里得距离
            double dist = sqrt(pow(task.x - center.x, 2) + pow(task.y - center.y, 2));
            
            // 更新最近的配送中心
            if (dist < minDist) {
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
