#include "common.h"
#include <iostream>
#include <iomanip>  // 用于设置输出精度

using std::vector;
using std::sqrt;
using std::ifstream;
using std::cout;
using std::endl;
using std::string;

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
            
            problem.network.edges[i] = {node1, node2, length};
            problem.network.distances[node1][node2] = length;
            problem.network.distances[node2][node1] = length;
        }

        // 使用 Floyd-Warshall 算法计算所有点对最短路径
        floydWarshall(problem.network);

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
                    vehicleIdCounter++,
                    vehicleSpeed,
                    vehicleCost,
                    0.0,    // maxLoad=0表示普通车辆
                    0.0,    // 普通车辆无燃料限制
                    id
                });
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
                    vehicleIdCounter++,
                    droneSpeed,
                    droneCost,
                    DeliveryProblem::DEFAULT_DRONE_LOAD,
                    DeliveryProblem::DEFAULT_DRONE_FUEL,
                    id
                });
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
        
        return true;
    }
    catch (const std::exception &e)
    {
        cout << "读取文件时发生错误: " << e.what() << endl;
        return false;
    }
}

// Floyd-Warshall 算法计算所有点对最短路径
void floydWarshall(RouteNetwork &network)
{
    // 初始化距离矩阵
    auto &dist = network.distances;
    
    // Floyd-Warshall 算法
    for (const auto &k : dist) {
        for (const auto &i : dist) {
            for (const auto &j : dist) {
                if (dist[i.first].count(k.first) && dist[k.first].count(j.first)) {
                    double throughK = dist[i.first][k.first] + dist[k.first][j.first];
                    if (!dist[i.first].count(j.first) || dist[i.first][j.first] > throughK) {
                        dist[i.first][j.first] = throughK;
                    }
                }
            }
        }
    }
}

// 计算两点间距离
double getDistance(int id1, int id2, const DeliveryProblem& problem, bool isDrone)
{
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