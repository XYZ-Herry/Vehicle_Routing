#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <fstream>
#include <unordered_map>
#include <ctime>  // 添加时间相关头文件
#include <unistd.h>
#include <unordered_set>

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
    double x, y;                // 坐标（公里）
    double time;                // 出现时间（小时）
    int centerId;               // 所属配送中心ID
    double pickweight;          // 取货重量（千克）
    double sendWeight;          // 送货重量（千克）
};

// 车辆/无人机结构体
struct Vehicle
{
    int id;                     // 车辆ID
    double speed;               // 速度（公里/小时）
    double cost;                // 单位距离成本
    double maxLoad;             // 最大载重（千克，0表示普通车辆）
    double maxfuel;                // 电池最大容量（小时，仅用于无人机）
    int centerId;               // 所属配送中心ID
};

// 配送中心结构体
struct DistributionCenter
{
    int id;                     // 配送中心ID
    double x, y;                // 配送中心坐标
    int vehicleCount;           // 普通车辆数量
    int droneCount;             // 无人机数量
    std::vector<int> vehicles;  // 所属车辆/无人机的ID列表
};

// 边结构体
struct Edge
{
    int node1;                  // 起点ID
    int node2;                  // 终点ID
    double length;              // 边长度
    double morningPeakFactor;   // 早高峰速度系数（7:00-9:00）
    double eveningPeakFactor;   // 晚高峰速度系数（17:00-18:00）
};

// 路网信息
struct RouteNetwork
{
    std::unordered_map<int, std::unordered_map<int, double>> distances; // 存储节点间最短距离
    std::vector<Edge> edges;                                            // 存储边信息
    
    // 存储路段的高峰期系数信息
    std::unordered_map<int, std::unordered_map<int, std::pair<double, double>>> peakFactors; // 存储节点间高峰期系数 (morningFactor, eveningFactor)
};

// 配送问题信息
struct DeliveryProblem
{
    // 默认值定义
    static constexpr double DEFAULT_DRONE_FUEL = 1.0;   // 默认无人机电池容量（小时）
    static constexpr double DEFAULT_DRONE_LOAD = 10.0;   // 默认无人机最大载重（千克）
    static constexpr int DEFAULT_CENTER_ID = -1;        // 默认配送中心ID

    std::vector<TaskPoint> tasks;                       // 所有任务点
    std::vector<Vehicle> vehicles;                      // 所有车辆（包括无人机）
    std::vector<DistributionCenter> centers;            // 所有配送中心
    RouteNetwork network;                               // 路网
    double timeWeight;                                  // 时间权重
    int initialDemandCount;                            // 初始需求点数量
    int extraDemandCount;                              // 额外需求点数量
    
    
    std::unordered_map<int, std::pair<double, double>> coordinates; // 存储所有点的坐标（ID -> 坐标）
    std::unordered_map<int, std::vector<int>> centerToTasks; // 存储每个中心ID分配的任务ID列表
    // 高峰时段参数
    double morningPeakFactor = 0.3;  // 早高峰速度系数（7:00-9:00）
    double eveningPeakFactor = 0.3;  // 晚高峰速度系数（17:00-18:00）

    // ID 到索引的映射
    std::unordered_map<int, int> centerIdToIndex;    // 中心ID到索引的映射
    std::unordered_map<int, int> taskIdToIndex;      // 任务ID到索引的映射
    std::unordered_map<int, int> vehicleIdToIndex;   // 车辆ID到索引的映射
    
    // 索引到ID的反向映射（如果需要）
    std::unordered_map<int, int> indexToTaskId;      // 索引到任务ID的映射
    std::unordered_map<int, int> indexToVehicleId;   // 索引到车辆ID的映射
    std::unordered_map<int, int> indexToCenterId;    // 索引到中心ID的映射
    
    // 所有配送中心ID的集合
    std::unordered_set<int> centerIds;
};

// 工具函数声明
bool loadProblemData(const std::string &filename, DeliveryProblem &problem);
double getDistance(int id1, int id2, const DeliveryProblem& problem, bool isDrone);
void floyd(RouteNetwork &network);

// 判断配送中心类型的辅助函数
inline bool isDroneCenter(const DistributionCenter& center) {
    return center.droneCount > 0;
}

inline bool isVehicleCenter(const DistributionCenter& center) {
    return center.vehicleCount > 0;
}

// 输出配送结果
void printDeliveryResults(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& allPaths);

// 输出动态阶段配送结果 - 更新参数类型
void printDynamicResults(
    const DeliveryProblem& problem, 
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths);

// 打印初始信息
void printInitialInfo(const DeliveryProblem& problem);

// 打印配送中心的分配情况
void printCenterAssignments(const DeliveryProblem& problem);

#endif