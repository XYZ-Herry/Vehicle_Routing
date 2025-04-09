#include "common.h"
#include "solver.h"  // 添加这一行以访问calculateTotalTimeAndCost函数
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

double droneSpeed, carSpeed, droneCost, vehicleCost, droneMaxLoad, droneMaxFuel, timeWeight;

// 从文件加载问题数据
bool loadProblemData(const string &filename, DeliveryProblem &problem)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
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
        file >> droneSpeed >> carSpeed >> droneCost >> vehicleCost >> droneMaxLoad >> droneMaxFuel >> timeWeight;
        problem.timeWeight = timeWeight;
        droneMaxFuel = DeliveryProblem::DEFAULT_DRONE_FUEL;
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
            
            
            problem.network.edges[i] = {node1, node2, length};
            problem.network.distances[node1][node2] = length;
            problem.network.distances[node2][node1] = length;
            
        }

        // 使用 Floyd算法计算所有点对最短路径
        floyd(problem.network);

        // 初始化任务点容器
        problem.tasks.resize(initialDemandCount + extraDemandCount);

        // 读取初始需求点
        for (int i = 0; i < initialDemandCount; ++i)
        {
            int id;
            double latitude, longitude, pickup_weight, delivery_weight;
            file >> id >> longitude >> latitude >> pickup_weight >> delivery_weight;
            auto [x, y] = convertLatLongToXY(latitude, longitude);
            problem.tasks[i] = {
                id, x, y, 0.0,    
                DeliveryProblem::DEFAULT_CENTER_ID,
                pickup_weight,
                delivery_weight
            };
            problem.coordinates[id] = {x, y};  // 存储坐标映射
        }

        problem.centers.resize(vehicleCenterCount + droneCenterCount);
        int vehicleIdCounter = 1;  // 从1开始
        
        for (int i = 0; i < vehicleCenterCount; ++i)
        {
            int id;
            double latitude, longitude;
            int vehicleCount;
            
            file >> id >> longitude >> latitude >> vehicleCount;
            id += 20000;//配送中心ID从20000开始
            auto [x, y] = convertLatLongToXY(latitude, longitude);
            
            problem.centers[i] = {id, x, y, vehicleCount, 0};
            problem.coordinates[id] = {x, y};  // 存储坐标映射
            
            // 创建车辆
            for (int j = 0; j < vehicleCount; ++j)
            {
                problem.vehicles.push_back({
                    vehicleIdCounter,//车辆ID
                    carSpeed,
                    vehicleCost,
                    0.0,    // maxLoad=0表示普通车辆
                    0.0,    // 普通车辆无燃料限制
                    id //这里的id是配送中心ID
                });
                problem.centers[i].vehicles.push_back(vehicleIdCounter); // 记录配送中心索引i内的车辆ID
                problem.allCarIds.push_back(vehicleIdCounter);
                vehicleIdCounter++;
            }
        }

        // 读取无人机配送中心
        for (int i = 0; i < droneCenterCount; ++i)
        {
            int id;
            double latitude, longitude;
            int droneCount;
            
            file >> id >> longitude >> latitude >> droneCount;
            id += 20000;
            auto [x, y] = convertLatLongToXY(latitude, longitude);

            problem.centers[vehicleCenterCount + i] = {id, x, y, 0, droneCount};
            problem.coordinates[id] = {x, y};  // 存储坐标映射
            
            // 创建无人机
            for (int j = 0; j < droneCount; ++j)
            {
                problem.vehicles.push_back({
                    vehicleIdCounter,
                    droneSpeed,
                    droneCost,
                    droneMaxLoad,
                    droneMaxFuel,
                    id
                });
                problem.centers[vehicleCenterCount + i].vehicles.push_back(vehicleIdCounter); // 记录无人机ID
                problem.allDroneIds.push_back(vehicleIdCounter);
                vehicleIdCounter++;
            }
        }

        // 读取额外需求点
        for (int i = 0; i < extraDemandCount; ++i)
        {
            int id;
            double latitude, longitude, pickup_weight, delivery_weight, arrivaltime;
            file >> id >> longitude >> latitude >> pickup_weight >> delivery_weight >> arrivaltime;
            arrivaltime = arrivaltime / 60.0;//将分钟转换为小时
            auto [x, y] = convertLatLongToXY(latitude, longitude);
            
            // 给额外需求点添加偏移量确保ID唯一
            int uniqueId = id + 10000;  // 假设原始ID不会超过10000
            
            problem.tasks[initialDemandCount + i] = {
                uniqueId, x, y, arrivaltime,
                DeliveryProblem::DEFAULT_CENTER_ID,
                pickup_weight,
                delivery_weight
            };
            problem.coordinates[uniqueId] = {x, y};  // 存储坐标映射
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
            problem.centerIds.insert(problem.centers[i].id); // 添加到集合中
        }
        
        for (size_t i = 0; i < problem.tasks.size(); i++) {
            problem.taskIdToIndex[problem.tasks[i].id] = i;
        }
        
        for (size_t i = 0; i < problem.vehicles.size(); i++) {
            problem.vehicleIdToIndex[problem.vehicles[i].id] = i;
        }
        
        // 读取早高峰和晚高峰速度系数
        int node1, node2;
        double morningFactor, eveningFactor;
        while (file >> node1 >> node2 >> morningFactor >> eveningFactor) {
            // 存储早高峰和晚高峰速度系数
            for (int x = 0; x <= 2; x ++ ){
                for (int y = 0; y <= 2; y ++ ){
                    problem.network.peakFactors[node1 + x * 10000][node2 + y * 10000] = {morningFactor, eveningFactor};
                    problem.network.peakFactors[node2 + y * 10000][node1 + x * 10000] = {morningFactor, eveningFactor};  // 双向存储
                }
            }
        }
        
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "读取文件时发生错误: " << e.what() << std::endl;
        return false;
    }
}

// Floyd算法计算所有点对最短路径
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

    // 标准的Floyd算法
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
    
    if (id1 > 30000) id1 -= 30000;
    if (id2 > 30000) id2 -= 30000;
    
    if (isDrone) {
        // 获取两点坐标
        const auto& [x1, y1] = problem.coordinates.at(id1);
        const auto& [x2, y2] = problem.coordinates.at(id2);
        
        // 计算欧几里得距离
        double euclideanDist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        return euclideanDist;
    } else {
        // 车辆优先使用路网最短距离
        if (id1 > 20000) id1 -= 20000;
        if (id2 > 20000) id2 -= 20000;
        if (id1 > 10000) id1 -= 10000;
        if (id2 > 10000) id2 -= 10000;
        if (problem.network.distances.count(id1) && problem.network.distances.at(id1).count(id2)) {
            return problem.network.distances.at(id1).at(id2);
        }
 
        return std::numeric_limits<double>::infinity();//如果路网中没有这两点间的距离，设置为无穷大
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

// 打印初始阶段的信息
void printInitialInfo(const DeliveryProblem& problem) {
    std::cout << "========== 初始阶段信息 ==========" << std::endl;
    
    // 输出基本参数
    std::cout << "Car速度: " << carSpeed << " km/h" << std::endl;
    std::cout << "Drone速度: " << droneSpeed << " km/h" << std::endl;
    std::cout << "Drone载重: " << droneMaxLoad << " kg" << std::endl;
    std::cout << "Drone电量: " << droneMaxFuel << " h" << std::endl;
    std::cout << "时间权重: " << problem.timeWeight << std::endl;
    std::cout << "延迟任务惩罚系数: " << DeliveryProblem::DEFAULT_DELAY_PENALTY << std::endl;
    std::cout << "早高峰时间: [" << DeliveryProblem::MORNING_PEAK_START << ", " 
              << DeliveryProblem::MORNING_PEAK_END << "], 默认速度系数: " 
              << problem.morningPeakFactor << std::endl;
    std::cout << "晚高峰时间: [" << DeliveryProblem::EVENING_PEAK_START << ", " 
              << DeliveryProblem::EVENING_PEAK_END << "], 默认速度系数: " 
              << problem.eveningPeakFactor << std::endl;
    
    // 输出配送中心信息
    std::cout << "配送中心数量: " << problem.centers.size() << "个" << std::endl;
    for (const auto& center : problem.centers) {
        // 确定配送中心类型
        std::string centerType;
        if (center.carCount > 0 && center.droneCount > 0) {
            centerType = "混合配送中心";
        } else if (center.carCount > 0) {
            centerType = "Car配送中心";
        } else if (center.droneCount > 0) {
            centerType = "Drone配送中心";
        } else {
            centerType = "未知配送中心";
        }
        
        std::cout << centerType << " ID: " << center.id << ", 坐标: (" << center.x << ", " << center.y 
                  << "), 所含car/drone ID: ";
        for (size_t i = 0; i < center.vehicles.size(); ++i) {
            if (i > 0) std::cout << ", ";
            std::cout << center.vehicles[i];
        }
        std::cout << std::endl;
    }
    
    // 输出任务点信息
    std::cout << "任务点数量: " << problem.tasks.size() << "个" << std::endl;
    std::cout << "初始任务点数量: " << problem.initialDemandCount << "个" << std::endl;
    for (size_t i = 0; i < problem.initialDemandCount; ++i) {
        const auto& task = problem.tasks[i];
        std::cout << "任务点坐标: (" << task.x << ", " << task.y << "), ID: " << task.id 
                  << ", 取货重量: " << task.pickweight << ", 送货重量: " << task.sendWeight << std::endl;
    }
    
    // 输出额外任务点信息
    int extraCount = problem.tasks.size() - problem.initialDemandCount;
    std::cout << "额外任务点数量: " << extraCount << "个" << std::endl;
    for (size_t i = problem.initialDemandCount; i < problem.tasks.size(); ++i) {
        const auto& task = problem.tasks[i];
        std::cout << "任务点坐标: (" << task.x << ", " << task.y << "), ID: " << task.id 
                  << ", 到达时间: " << task.arrivaltime << "h, 取货重量: " << task.pickweight 
                  << ", 送货重量: " << task.sendWeight << std::endl;
    }
    
    std::cout << "========== 初始阶段信息结束 ==========" << std::endl;
}

void Print_DeliveryResults(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& allPaths)
{
    cout << "\n=== 具体配送路径与时间 ===" << endl;
    double totalTaskCount = 0;
    
    for (const auto& [vehicleId, pathData] : allPaths) {
        const auto& [path, completionTimes] = pathData;
        if (path.size() <= 2) continue;  // 跳过无任务的路径
        
        // 查找车辆索引
        int vehicleIndex = problem.vehicleIdToIndex.at(vehicleId);
        const auto& vehicle = problem.vehicles[vehicleIndex];
        
        // 使用英文统一输出车辆类型
        bool isDrone = (vehicle.maxLoad > 0);
        cout << (isDrone ? "Drone" : "Car") << " #" << vehicleId << " 的路径: ";
        
        // 输出路径上的每个点
        for (size_t i = 0; i < path.size(); ++i) {
            int pointId = path[i];
            
            // 区分配送中心和任务点还有车机协同的车辆任务点
            if (problem.centerIds.count(pointId) > 0) {
                cout << "中心#" << pointId;
            }
            else if (pointId > 30000){
                cout << "协同点#" << pointId;
            }
            else{
                cout << "任务#" << pointId;
                totalTaskCount++;
            }
            
            if (i < path.size() - 1) {
                cout << " -> ";
            }
        }
        cout << endl;
        
        // 打印完成时间
        if (completionTimes.size() >= 2) {
            cout << "  完成时间: ";
            for (size_t i = 0; i < completionTimes.size(); ++i) {
                cout << " " << completionTimes[i] << "h";
                if (i < completionTimes.size() - 1) {
                    cout << ",";
                }
            }
            cout << endl;
        }
    }
    
    cout << "\n总共配送任务数: " << totalTaskCount << endl;
    
    // 计算总时间和成本
    auto [totalTime, totalCost] = calculateTotalTimeAndCost(problem, allPaths);
    cout << "所有任务的最晚完成时间: " << totalTime << " 小时" << endl;
    cout << "总成本: " << totalCost << " 元" << endl;
}

// 输出配送中心的车辆分配和任务分配情况
void printCenterAssignments(const DeliveryProblem& problem) {
    cout << "配送中心任务分配结果：" << endl;
    
    // 创建配送中心到车辆的映射
    std::unordered_map<int, std::vector<int>> centerToVehicles;
    for (const auto& vehicle : problem.vehicles) {
        centerToVehicles[vehicle.centerId].push_back(vehicle.id);
    }
    
    // 输出每个中心的车辆分配和任务分配
    for (const auto& center : problem.centers) {
        cout << "配送中心 #" << center.id << " (";
        
        // 判断配送中心类型
        bool hasDrones = false;
        bool hasTrucks = false;
        for (int vehicleId : centerToVehicles[center.id]) {
            int vIndex = problem.vehicleIdToIndex.at(vehicleId);
            if (problem.vehicles[vIndex].maxLoad > 0)
                hasDrones = true;
            else
                hasTrucks = true;
        }
        
        if (hasDrones && hasTrucks)
            cout << "混合配送中心";
        else if (hasDrones)
            cout << "Drone配送中心";
        else
            cout << "Car配送中心";
        
        cout << "): " << centerToVehicles[center.id].size() << " 个车辆，分别是：";
        
        for (size_t i = 0; i < centerToVehicles[center.id].size(); ++i) {
            if (i > 0) cout << ", ";
            cout << centerToVehicles[center.id][i];
        }
        cout << endl;
        
        // 输出任务分配情况
        auto tasksIt = problem.centerToTasks.find(center.id);
        if (tasksIt != problem.centerToTasks.end() && !tasksIt->second.empty()) {
            cout << "  分配任务数: " << tasksIt->second.size() << "，任务IDs: ";
            for (size_t i = 0; i < std::min(tasksIt->second.size(), size_t(10)); ++i) {
                if (i > 0) cout << ", ";
                cout << tasksIt->second[i];
            }
            if (tasksIt->second.size() > 10) {
                cout << "...";
            }
            cout << endl;
        } else {
            cout << "  没有分配任务" << endl;
        }
    }
}

