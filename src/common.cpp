#include "common.h"
#include <iostream>
#include <iomanip>  // 用于设置输出精度
#include <algorithm>  // 添加对 std::sort 的包含
#include <unordered_map>  // 添加对 std::unordered_map 的包含
#include <vector>
using std::vector;
using std::sqrt;
using std::ifstream;
using std::cout;
using std::endl;
using std::string;
using std::unordered_map;

// 从文件加载问题数据
bool loadProblemData(const string &filename, DeliveryProblem &problem)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        cout << "无法打开文件: " << filename << endl;
        return false;
    }

    try
    {
        // 读取基本问题参数
        int initialDemandCount, extraDemandCount, vehicleCenterCount, droneCenterCount;
        file >> initialDemandCount >> extraDemandCount >> vehicleCenterCount >> droneCenterCount;
        problem.initialDemandCount = initialDemandCount;
        problem.extraDemandCount = extraDemandCount;

        // 读取车辆和无人机的参数
        double droneSpeed, vehicleSpeed, droneCost, vehicleCost, droneMaxLoad, timeWeight;
        file >> droneSpeed >> vehicleSpeed >> droneCost >> vehicleCost >> droneMaxLoad >> timeWeight;
        problem.timeWeight = timeWeight;

        // 读取路网信息
        int edgeCount;
        file >> edgeCount;
        problem.network.edges.resize(edgeCount);

        // 构建路网的边和距离矩阵
        for (int i = 0; i < edgeCount; ++i)
        {
            int node1, node2;
            double length;
            file >> node1 >> node2 >> length;
            
            // 将米转换为公里
            length /= 1000.0;
            
            // 默认的高峰期系数，如果输入文件中没有提供
            double morningPeakFactor = 0.3;
            double eveningPeakFactor = 0.3;
            
            // 如果输入文件提供了高峰期系数
            // if (file.peek() != '\n') {
            //     file >> morningPeakFactor >> eveningPeakFactor;
            // }
            
            problem.network.edges[i] = {node1, node2, length, morningPeakFactor, eveningPeakFactor};
            problem.network.distances[node1][node2] = length;
            problem.network.distances[node2][node1] = length;
            
            // 存储高峰期系数
            problem.network.peakFactors[node1][node2] = {morningPeakFactor, eveningPeakFactor};
            problem.network.peakFactors[node2][node1] = {morningPeakFactor, eveningPeakFactor}; // 假设道路是双向的
        }

        // 使用 Floyd-Warshall 算法计算所有点对最短路径
        floyd(problem.network);

        // 确保每个节点到自身的距离为0
        for (const auto& [nodeId, _] : problem.network.distances) {
            if (problem.network.distances.count(nodeId)) {
                problem.network.distances.at(nodeId)[nodeId] = 0.0;
            }
        }

        // 初始化任务点容器
        problem.tasks.resize(initialDemandCount + extraDemandCount);

        cout << "\n============= 坐标信息 =============" << endl;
        cout << std::fixed << std::setprecision(3);  // 设置输出精度

        // 读取初始需求点
        cout << "\n初始需求点坐标:" << endl;
        for (int i = 0; i < initialDemandCount; ++i)
        {
            int id;
            double latitude, longitude;
            file >> id >> longitude >> latitude;
            auto [x, y] = convertLatLongToXY(latitude, longitude);
            problem.tasks[i] = {
                id, x, y, 0.0,    
                DeliveryProblem::DEFAULT_CENTER_ID,
                1.0  // 默认重量1kg
            };
            problem.coordinates[id] = {x, y};  // 存储坐标映射
            cout << "任务点 " << id << ": (" << x << " km, " << y << " km)" << endl;
        }

        // 读取车辆配送中心
        cout << "\n车辆配送中心坐标:" << endl;
        problem.centers.resize(vehicleCenterCount + droneCenterCount);
        int vehicleIdCounter = 0;
        
        for (int i = 0; i < vehicleCenterCount; ++i)
        {
            int id;
            double latitude, longitude;
            int vehicleCount;
            file >> id >> longitude >> latitude >> vehicleCount;
            auto [x, y] = convertLatLongToXY(latitude, longitude);
            
            problem.centers[i] = {id, x, y, vehicleCount, 0};
            problem.coordinates[id] = {x, y};  // 存储坐标映射
            cout << "配送中心 " << id << ": (" << x << " km, " << y << " km), " 
                 << vehicleCount << " 辆车" << endl;
            
            // 创建车辆
            for (int j = 0; j < vehicleCount; ++j)
            {
                problem.vehicles.push_back({
                    vehicleIdCounter,
                    vehicleSpeed,
                    vehicleCost,
                    0.0,    // maxLoad=0表示普通车辆
                    0.0,    // 普通车辆无燃料限制
                    id
                });
                problem.centers[i].vehicles.push_back(vehicleIdCounter); // 记录车辆ID
                vehicleIdCounter++;
            }
        }

        // 读取无人机配送中心
        cout << "\n无人机配送中心坐标:" << endl;
        for (int i = 0; i < droneCenterCount; ++i)
        {
            int id;
            double latitude, longitude;
            int droneCount;
            file >> id >> longitude >> latitude >> droneCount;
            auto [x, y] = convertLatLongToXY(latitude, longitude);

            problem.centers[vehicleCenterCount + i] = {id, x, y, 0, droneCount};
            problem.coordinates[id] = {x, y};  // 存储坐标映射
            cout << "配送中心 " << id << ": (" << x << " km, " << y << " km), " 
                 << droneCount << " 架无人机" << endl;
            
            // 创建无人机
            for (int j = 0; j < droneCount; ++j)
            {
                problem.vehicles.push_back({
                    vehicleIdCounter,
                    droneSpeed,
                    droneCost,
                    DeliveryProblem::DEFAULT_DRONE_LOAD,
                    DeliveryProblem::DEFAULT_DRONE_FUEL,
                    id
                });
                problem.centers[vehicleCenterCount + i].vehicles.push_back(vehicleIdCounter); // 记录无人机ID
                vehicleIdCounter++;
            }
        }

        // 读取额外需求点
        cout << "\n额外需求点坐标:" << endl;
        for (int i = 0; i < extraDemandCount; ++i)
        {
            int id;
            double latitude, longitude, time;
            file >> id >> longitude >> latitude >> time;
            auto [x, y] = convertLatLongToXY(latitude, longitude);
            problem.tasks[initialDemandCount + i] = {
                id, x, y, time,
                DeliveryProblem::DEFAULT_CENTER_ID,
                1.0  // 默认重量1kg
            };
            problem.coordinates[id] = {x, y};  // 存储坐标映射
            cout << "任务点 " << id << ": (" << x << " km, " << y << " km)" << endl;
        }

        cout << "\n=====================================" << endl;
        cout << "数据加载成功！" << endl;
        cout << "初始需求点: " << initialDemandCount << ", 额外需求点: " << extraDemandCount << endl;
        cout << "车辆数量: " << problem.vehicles.size() << endl;
        
        int droneCount = 0;
        for (const auto& vehicle : problem.vehicles) {
            if (vehicle.maxLoad > 0) droneCount++;
        }
        cout << "其中无人机数量: " << droneCount << endl;
        
        // 在读取中心数据后添加
        for (size_t i = 0; i < problem.centers.size(); ++i) {
            problem.centerIdToIndex[problem.centers[i].id] = i;
        }
        
        return true;
    }
    catch (const std::exception &e)
    {
        cout << "读取文件时发生错误: " << e.what() << endl;
        return false;
    }
}

// Floyd-Warshall 算法计算所有点对最短路径
void floyd(RouteNetwork &network)
{
    // 收集所有节点ID
    std::vector<int> nodes;
    for (const auto& pair : network.distances) {
        nodes.push_back(pair.first);
    }
    
    // 节点数量
    int n = nodes.size();

    // 初始化距离矩阵 (确保每个点到自身的距离为0)
    for (int i = 0; i < n; i++) {
        if (network.distances.count(nodes[i])) {
            network.distances.at(nodes[i])[nodes[i]] = 0.0;
        }
    }

    // 标准的Floyd-Warshall算法
    for (int k = 0; k < n; k++) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (network.distances.count(nodes[i]) && 
                    network.distances[nodes[i]].count(nodes[k]) &&
                    network.distances.count(nodes[k]) && 
                    network.distances[nodes[k]].count(nodes[j])) {
                
                    double throughK = network.distances[nodes[i]][nodes[k]] + 
                                    network.distances[nodes[k]][nodes[j]];
                    
                    if (!network.distances[nodes[i]].count(nodes[j]) || 
                        throughK < network.distances[nodes[i]][nodes[j]]) {
                        network.distances[nodes[i]][nodes[j]] = throughK;
                    }
                }
            }
        }
    }
}

// 计算两点间距离
double getDistance(int id1, int id2, const DeliveryProblem& problem, bool isDrone)
{
    // 同一个点到自身的距离应为0
    if (id1 == id2) {
        return 0.0;
    }
    
    // 获取两点坐标
    const auto& [x1, y1] = problem.coordinates.at(id1);
    const auto& [x2, y2] = problem.coordinates.at(id2);
    
    // 计算欧几里得距离
    double euclideanDist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    
    if (isDrone) {
        return euclideanDist;
    } else {
        // 车辆优先使用路网最短距离
        if (problem.network.distances.count(id1) && problem.network.distances.at(id1).count(id2)) {
            return problem.network.distances.at(id1).at(id2);
        }
        //return euclideanDist * 1.2;  // 如果路网中没有这两点间的距离，使用欧几里得距离的1.2倍
        return std::numeric_limits<double>::infinity();
    }
}

// 经纬度转换为直角坐标系（单位：公里）
std::pair<double, double> convertLatLongToXY(double latitude, double longitude) {
    // 将经纬度转换为弧度
    double lat = latitude * DEG_TO_RAD;
    double lon = longitude * DEG_TO_RAD;
    
    // 转换为直角坐标（使用墨卡托投影）
    double x = EARTH_RADIUS * lon;
    double y = EARTH_RADIUS * log(tan(M_PI/4 + lat/2));
    
    return {x, y};
}

// 输出配送中心到任务点的距离
void printCenterToTaskDistances(const DeliveryProblem& problem) {
    cout << "\n================== 配送中心到任务点距离信息 ==================" << endl;
    
    // 收集所有中心ID并排序（不需要额外建映射）
    std::vector<int> sortedCenterIds;
    for (const auto& center : problem.centers) {
        sortedCenterIds.push_back(center.id);
    }
    std::sort(sortedCenterIds.begin(), sortedCenterIds.end());

    // 遍历每个配送中心
    for (int centerId : sortedCenterIds) {
        // 查找配送中心信息
        const DistributionCenter* center = nullptr;
        for (const auto& c : problem.centers) {
            if (c.id == centerId) {
                center = &c;
                break;
            }
        }
        
        if (!center) continue;
        
        cout << "\n▶ 配送中心 " << centerId << " (";
        cout << (isDroneCenter(*center) ? "无人机中心" : "车辆中心") << "):" << endl;
        
        // 直接遍历该中心的所有车辆
        for (int vehicleId : center->vehicles) {
            if (problem.centerAssignments.count(vehicleId)) {
                const auto& assignment = problem.centerAssignments.at(vehicleId);
                const auto& path = assignment.first;
                const auto& completionTimes = assignment.second;
                const Vehicle& vehicle = problem.vehicles[vehicleId];
                
                if (path.empty() || path.size() <= 2) continue; // 跳过未分配任务的车辆
                
                cout << "  车辆 " << std::setw(3) << vehicleId << " (";
                if (vehicle.maxLoad > 0) {
                    cout << "无人机, 载重: " << vehicle.maxLoad << "kg, 电池: " << vehicle.fuel << "h";
                } else {
                    cout << "普通车辆";
                }
                cout << "):" << endl;
                
                // 输出配送路径序列
                cout << "    任务序列: ";
                for (size_t j = 0; j < path.size(); ++j) {
                    if (j > 0 && j < path.size() - 1) { // 跳过起点和终点的配送中心
                        cout << path[j];
                        if (j < path.size() - 1) cout << " → ";
                    }
                }
                cout << endl;
                
                // 输出每个任务的完成时间
                if (!completionTimes.empty()) {
                    cout << "    完成时间: ";
                    for (double time : completionTimes) {
                        cout << std::fixed << std::setprecision(3) << std::setw(7) << time << " ";
                    }
                    cout << endl;
                }
                
                // 计算成本
                int tasksDelivered = path.size() - 2; // 减去起点和终点的配送中心
                double vehicleCost = tasksDelivered * vehicle.cost;
                cout << "    配送任务数: " << tasksDelivered << ", 成本: " << vehicleCost << endl;
            }
        }
    }
    cout << "\n=============================================================" << endl;
    // std::cout << problem.network.distances.count(101) << ' ' <<  problem.network.distances.at(101).count(140) << "\n";
    // std::cout << problem.network.distances.at(101).at(140) << "\n";
}

// 输出配送结果
void printDeliveryResults(const DeliveryProblem& problem, 
                         const std::vector<std::pair<std::vector<int>, std::vector<double>>>& allPaths) {
    cout << "\n==================== 配送结果详情 ====================" << endl;
    
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    int totalTasksDelivered = 0;
    
    // 收集所有中心ID并排序（不需要额外建映射）
    std::vector<int> sortedCenterIds;
    for (const auto& center : problem.centers) {
        sortedCenterIds.push_back(center.id);
    }
    std::sort(sortedCenterIds.begin(), sortedCenterIds.end());

    // 遍历每个配送中心
    for (int centerId : sortedCenterIds) {
        // 查找配送中心信息
        const DistributionCenter* center = nullptr;
        for (const auto& c : problem.centers) {
            if (c.id == centerId) {
                center = &c;
                break;
            }
        }
        
        if (!center) continue;
        
        cout << "\n▶ 配送中心 " << centerId << " (";
        cout << (isDroneCenter(*center) ? "无人机中心" : "车辆中心") << "):" << endl;
        
        // 直接遍历该中心的所有车辆
        for (int vehicleId : center->vehicles) {
            const auto& [path, completionTimes] = allPaths[vehicleId];
            const Vehicle& vehicle = problem.vehicles[vehicleId];
            
            if (path.empty() || path.size() <= 2) continue; // 跳过未分配任务的车辆
            
            cout << "  车辆 " << std::setw(3) << vehicleId << " (";
            if (vehicle.maxLoad > 0) {
                cout << "无人机, 载重: " << vehicle.maxLoad << "kg, 电池: " << vehicle.fuel << "h";
            } else {
                cout << "普通车辆";
            }
            cout << "):" << endl;
            
            // 输出配送路径序列
            cout << "    任务序列: ";
            for (size_t j = 0; j < path.size(); ++j) {
                if (j > 0 && j < path.size() - 1) { // 跳过起点和终点的配送中心
                    totalTasksDelivered++;
                }
                cout << path[j];
                if (j < path.size() - 1) cout << " → ";
            }
            cout << endl;
            
            // 输出每个任务的完成时间
            if (!completionTimes.empty()) {
                cout << "    完成时间: ";
                for (double time : completionTimes) {
                    cout << std::fixed << std::setprecision(3) << std::setw(7) << time << " ";
                    maxCompletionTime = std::max(maxCompletionTime, time);
                }
                cout << endl;
            }
            
            // 计算成本
            int tasksDelivered = path.size() - 2; // 减去起点和终点的配送中心
            double vehicleCost = tasksDelivered * vehicle.cost;
            totalCost += vehicleCost;
            
            cout << "    配送任务数: " << tasksDelivered << ", 成本: " << vehicleCost << endl;
        }
    }
    
    cout << "\n▶ 总结统计:" << endl;
    cout << "  总任务数: " << totalTasksDelivered << endl;
    cout << "  最大完成时间: " << std::fixed << std::setprecision(3) << maxCompletionTime << " 小时" << endl;
    cout << "  总配送成本: " << totalCost << endl;
    cout << "\n=============================================================" << endl;
}

// 输出动态阶段配送结果
void printDynamicResults(
    const DeliveryProblem& problem, 
    const std::vector<std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    const std::vector<std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths)
{
    cout << "\n=============================================================" << endl;
    cout << "▶ 动态阶段配送结果" << endl;
    cout << "=============================================================" << endl;
    
    // 统计不同指标
    int totalTasksDelivered = 0;
    double maxCompletionTime = 0.0;
    double totalCost = 0.0;
    
    // 收集所有中心ID并排序（不需要额外建映射）
    std::vector<int> sortedCenterIds;
    for (const auto& center : problem.centers) {
        sortedCenterIds.push_back(center.id);
    }
    std::sort(sortedCenterIds.begin(), sortedCenterIds.end());

    // 遍历每个配送中心
    for (int centerId : sortedCenterIds) {
        // 使用centerIdToIndex直接查找中心
        if (problem.centerIdToIndex.count(centerId)) {
            int centerIdx = problem.centerIdToIndex.at(centerId);
            const auto& center = problem.centers[centerIdx];
            
            cout << "\n▶ 配送中心 " << centerId << " (";
            cout << (isDroneCenter(center) ? "无人机中心" : "车辆中心") << "):" << endl;
            
            // 直接遍历该中心的所有车辆
            for (int vehicleId : center.vehicles) {
                const auto& [path, completionTimes] = dynamicPaths[vehicleId];
                const Vehicle& vehicle = problem.vehicles[vehicleId];
                
                if (path.empty() || path.size() <= 2) continue; // 跳过未分配任务的车辆
                
                cout << "  车辆 " << std::setw(3) << vehicleId << " (";
                if (vehicle.maxLoad > 0) {
                    cout << "无人机, 载重: " << vehicle.maxLoad << "kg, 电池: " << vehicle.fuel << "h";
                } else {
                    cout << "普通车辆";
                }
                cout << "):" << endl;
                
                // 输出配送路径序列
                cout << "    任务序列: ";
                for (size_t j = 0; j < path.size(); ++j) {
                    if (j > 0 && j < path.size() - 1) { // 跳过起点和终点的配送中心
                        totalTasksDelivered++;
                    }
                    cout << path[j];
                    if (j < path.size() - 1) cout << " → ";
                }
                cout << endl;
                
                // 输出每个任务的完成时间
                if (!completionTimes.empty()) {
                    cout << "    完成时间: ";
                    for (double time : completionTimes) {
                        cout << std::fixed << std::setprecision(3) << std::setw(7) << time << " ";
                        maxCompletionTime = std::max(maxCompletionTime, time);
                    }
                    cout << endl;
                }
                
                // 计算成本
                int tasksDelivered = path.size() - 2; // 减去起点和终点的配送中心
                double vehicleCost = tasksDelivered * vehicle.cost;
                totalCost += vehicleCost;
                
                cout << "    配送任务数: " << tasksDelivered << ", 成本: " << vehicleCost << endl;
                
                // 计算与静态阶段的对比
                const auto& [staticPath, staticTimes] = staticPaths[vehicleId];
                if (!staticPath.empty() && staticPath.size() > 2) {
                    double staticMaxTime = 0.0;
                    if (!staticTimes.empty()) {
                        staticMaxTime = staticTimes.back();
                    }
                    
                    double timeChange = completionTimes.empty() ? 0 : (completionTimes.back() - staticMaxTime);
                    int taskChange = (path.size() - 2) - (staticPath.size() - 2);
                    
                    cout << "    与静态阶段相比: 时间";
                    if (timeChange > 0) cout << "增加 " << std::fixed << std::setprecision(3) << timeChange;
                    else cout << "减少 " << std::fixed << std::setprecision(3) << -timeChange;
                    
                    cout << "小时, 任务";
                    if (taskChange > 0) cout << "增加 " << taskChange;
                    else if (taskChange < 0) cout << "减少 " << -taskChange;
                    else cout << "无变化";
                    cout << endl;
                }
            }
        }
    }
    
    cout << "\n▶ 总结统计:" << endl;
    cout << "  总任务数: " << totalTasksDelivered << endl;
    cout << "  最大完成时间: " << std::fixed << std::setprecision(3) << maxCompletionTime << " 小时" << endl;
    cout << "  总配送成本: " << totalCost << endl;
    cout << "\n=============================================================" << endl;
}