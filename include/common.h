#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <fstream>
#include <unordered_map>
#include <ctime>  // 添加时间相关头文件

// 前向声明
struct TaskPoint;
struct Vehicle;
struct DistributionCenter;
struct Edge;
struct RouteNetwork;
struct DeliveryProblem;

// 地球半径（单位：公里）
constexpr double EARTH_RADIUS = 6371.0;

// 经纬度转换为弧度
constexpr double DEG_TO_RAD = M_PI / 180.0;

// 经纬度转换为直角坐标系（单位：公里）
std::pair<double, double> convertLatLongToXY(double latitude, double longitude);

// 任务点结构体
struct TaskPoint
{
    int id;                     // 任务点ID
    double x, y;                // 任务点坐标
    double arrivalTime;         // 到达时间（>0表示额外需求点，=0表示初始需求点）
    int centerId;               // 所属配送中心ID
};

// 车辆/无人机结构体
struct Vehicle
{
    int id;                     // 车辆ID
    double speed;               // 速度（公里/时）
    double cost;                // 单个货物的运输成本（元）
    double maxLoad;             // 最大载重（kg）
    double fuel;                // 电池容量（时）
    int centerId;               // 所属配送中心ID
};

// 配送中心结构体
struct DistributionCenter
{
    int id;                     // 配送中心ID
    double x, y;                // 配送中心坐标
    int vehicleCount;           // 普通车辆数量
    int droneCount;             // 无人机数量
};

// 边结构体
struct Edge
{
    int node1;          // 起点ID
    int node2;          // 终点ID
    double length;      // 边长度
};

// 路网信息
struct RouteNetwork
{
    std::unordered_map<int, std::unordered_map<int, double>> distances; // 存储节点间最短距离
    std::vector<Edge> edges;                                            // 存储边信息
};

// 配送问题信息
struct DeliveryProblem
{
    // 默认值定义
    static constexpr double DEFAULT_DRONE_FUEL = 5.0;     // 无人机默认电池容量（时）
    static constexpr double DEFAULT_DRONE_LOAD = 50.0;    // 无人机默认最大载重（kg）
    static constexpr int DEFAULT_CENTER_ID = -1;           // 默认配送中心ID

    std::vector<TaskPoint> tasks;                    // 所有任务点列表
    std::vector<DistributionCenter> centers;         // 所有配送中心列表
    std::vector<Vehicle> vehicles;                   // 所有车辆和无人机列表
    RouteNetwork network;                            // 路网信息
    double timeWeight;                               // 时间权重（用于目标函数）
    int initialDemandCount;                          // 初始需求点数量
    int extraDemandCount;                            // 额外需求点数量
    std::vector<std::vector<int>> centerAssignments; // 每个配送中心分配的任务ID列表
};

// 工具函数声明

bool loadProblemData(const std::string &filename, DeliveryProblem &problem);

// 计算两点间距离的函数声明
double getDistance(const TaskPoint &a, const TaskPoint &b, const RouteNetwork &network, bool isDrone);
void floydWarshall(RouteNetwork &network);  // 添加 Floyd 算法函数声明

// 判断配送中心类型的辅助函数
inline bool isDroneCenter(const DistributionCenter& center) {
    return center.droneCount > 0;
}

inline bool isVehicleCenter(const DistributionCenter& center) {
    return center.vehicleCount > 0;
}

#endif