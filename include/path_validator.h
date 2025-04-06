#ifndef PATH_VALIDATOR_H
#define PATH_VALIDATOR_H

#include "common.h"
#include <vector>
#include <utility>
#include <unordered_map>
#include <string>

// 验证静态阶段路径中车辆/无人机是否属于一开始分到的配送中心
bool validateStaticVehicleCenter(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths);

// 验证动态阶段中未超时的车辆是否仍属于原配送中心
bool validateDynamicVehicleCenter(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths,
    double staticMaxTime);

// 验证静态阶段路径合法性(无人机电量、载重约束，时间计算)
std::pair<bool, std::string> validateStaticPathLegality(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths);

// 验证动态阶段路径合法性(考虑高峰期影响)
std::pair<bool, std::string> validateDynamicPathLegality(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths,
    const std::vector<int>& extraTaskIds);

std::pair<bool, std::string> validateStaticPathCompleteness(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths);

std::pair<bool, std::string> validateDynamicPathCompleteness(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths);

// 主验证函数
std::pair<bool, std::string> validateAllPaths(
    const DeliveryProblem& problem,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& staticPaths,
    const std::unordered_map<int, std::pair<std::vector<int>, std::vector<double>>>& dynamicPaths,
    double staticMaxTime,
    const std::vector<int>& extraTaskIds);

#endif // PATH_VALIDATOR_H 