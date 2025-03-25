

# 车辆路径规划系统算法介绍

## 1. 系统概述

本系统实现了一种复杂的车辆和无人机混合配送路径规划解决方案，分为静态阶段和动态阶段。静态阶段处理已知的任务需求，而动态阶段则处理额外需求点和由于交通状况变化导致的延迟任务。系统采用遗传算法进行任务分配优化，使用改进的最近邻算法进行路径规划，并实现了车辆与无人机的协同工作机制。

## 2. 系统算法流程

整个系统算法流程如下：

````
算法：配送路径规划系统主流程
输入：配送中心、车辆信息、任务点信息
输出：优化后的配送路径和时间

1. 数据预处理：读入并处理所有数据（common.cpp）
2. 任务初始分配：每个任务点分配到距离最近的配送中心（assignTasksToCenters函数）
3. 静态阶段遗传算法：按配送中心分别进行遗传算法，为各中心的车辆分配任务点（static_genetic.cpp）
4. 静态阶段路径优化：优化各车辆/无人机的路径并计算完成时间（optimizePathForVehicle函数）
5. 计算静态阶段最晚完成时间T（main.cpp里面）
6. 动态阶段任务识别：收集超过时间T的延迟任务和新增任务（solver.cpp里面的identifyTasksForRescheduling函数）
7. 动态阶段遗传算法：在静态最优解基础上重新分配任务点（dynamic_genetic.cpp）
8. 动态阶段路径优化：考虑车机协同优化所有路径（Dynamic_OptimizePathForVehicle和optimizeDronePathWithVehicles函数）
9. 返回并输出最终优化的路径和时间表（main.cpp里的Print_DeliveryResults函数）
````

## 3. 任务分配算法

初始任务分配将每个任务点分配给距离最近的配送中心，确定任务点的管辖范围。

````
算法：任务初始分配（assignTasksToCenters函数）
输入：配送中心集合 centers，任务点集合 tasks
输出：各配送中心的任务点映射 centerToTasks

1. 对每个任务点 task in tasks 执行:
   a. minDistance ← ∞
   b. nearestCenter ← -1
   c. 对每个配送中心 center in centers 执行:
      i. distance ← 计算任务点task到配送中心center的距离
      ii. 如果 distance < minDistance 则:
          minDistance ← distance
          nearestCenter ← center.id
   d. centerToTasks[nearestCenter].push(task.id)
   e. 更新任务点的所属中心: task.centerId ← nearestCenter
2. 返回 centerToTasks
````

## 4. 静态阶段优化

### 4.1 静态阶段遗传算法

静态阶段采用按配送中心划分的遗传算法，每个配送中心的车辆只负责本辖区内的任务点。

````
算法：静态阶段遗传算法（static_genetic.cpp）
输入：问题实例 problem，种群大小 populationSize，迭代代数 generations，
     变异率 mutationRate，时间权重 timeWeight
输出：任务分配方案 assignments

1. 初始化空的分配方案 assignments
2. 对每个配送中心 centerId in problem.centerIds 执行:
   a. 如果该配送中心没有任务点，跳过
   b. 找出属于该配送中心的所有车辆 centerVehicleIds
   c. 获取该配送中心负责的所有任务点 centerTaskIds
   d. 初始化种群 population，每个个体表示任务到车辆的分配方案
   e. 对每一代 gen from 0 to generations-1 执行:
      i. 创建新种群 newPopulation
      ii. 计算每个个体的适应度，存入 fitnessPopulation
      iii. 选择操作：从fitnessPopulation中选择较好的个体
      iv. 交叉操作：随机选择交叉点，交换部分基因
      v. 变异操作：以mutationRate的概率进行变异，将任务重新分配给同一中心的另一辆车
      vi. 保留可行解，更新种群 population
   f. 选择最优个体，将其分配方案添加到 assignments
3. 返回 assignments
4. 计算最晚完成时间T，用于动态阶段参考
````

### 4.2 静态阶段路径优化

根据分配结果，分别对普通车辆和无人机进行路径优化。

#### 4.2.1 普通车辆路径优化

普通车辆采用简单的贪心策略进行路径优化。

````
算法：普通车辆路径优化（optimizePathForVehicle函数）
输入：分配给车辆的任务ID列表 assignedTaskIds，任务点集合 tasks，车辆 vehicle，问题实例 problem
输出：优化后的路径 path

1. 若 assignedTaskIds 为空，返回 [vehicle.centerId, vehicle.centerId]
2. 初始化空路径 path，添加配送中心ID
3. 初始化已访问状态数组 visited，全部标记为未访问
4. 从配送中心开始，currentPos ← vehicle.centerId
5. 当仍有未访问的任务点时，执行:
   a. 初始化 minDistance ← ∞
   b. 对每个未访问的任务点 i 执行:
      i. 计算当前位置到任务点的距离 distance
      ii. 如果 distance < minDistance，更新 minDistance 和下一个任务点
   c. 如果找到下一个任务点，更新路径和已访问状态
   d. 否则结束循环
6. 路径结束后添加返回配送中心
7. 返回 path
````

#### 4.2.2 无人机路径优化

无人机路径优化需要考虑电量和载重约束。

````
算法：静态阶段无人机路径优化（optimizePathForVehicle函数中的无人机部分）
输入：分配给无人机的任务ID列表 assignedTaskIds，任务点集合 tasks，无人机 drone，问题实例 problem
输出：优化后的路径 path

1. 若 assignedTaskIds 为空，返回 [drone.centerId, drone.centerId]
2. 初始化路径 path，添加配送中心ID
3. 初始化已访问状态数组 visited，全部标记为未访问
4. 从配送中心开始，currentPos ← drone.centerId
5. 初始化电量 currentBattery ← drone.maxfuel，载重 currentLoad ← 0，过程最大载重 maxProcessLoad ← 0
6. 当仍有未访问的任务点时，执行:
   a. 初始化 minDistance ← ∞
   b. 对每个未访问的任务点 i 执行:
      i. 计算到任务点的距离 distance
      ii. 计算从任务点返回配送中心的距离 returnDistance
      iii. 检查电量约束：确保有足够电量到达该点并返回配送中心
      iv. 检查载重约束：
          - 如果是取货点，确保当前载重+取货重量≤最大载重
          - 如果是送货点，确保过程最大载重+送货重量≤最大载重
      v. 如果所有约束满足且 distance < minDistance，更新 minDistance 和下一个任务点
   c. 如果找到下一个任务点:
      i. 更新路径和已访问状态
      ii. 更新电量、当前载重和过程最大载重
      iii. 更新当前位置
   d. 否则:
      i. 返回配送中心
      ii. 重置电量、载重和过程最大载重
7. 返回 path
````

## 5. 动态阶段优化

### 5.1 动态阶段任务识别

动态阶段首先识别需要重新调度的任务，包括由于交通状况变化导致的延迟任务和新增的额外需求任务。

````
算法：动态阶段任务识别
输入：问题实例 problem，静态路径 staticPaths，静态阶段最晚完成时间 staticMaxTime
输出：延迟任务集合 delayedTasks，新增任务集合 newTasks

1. 初始化空的延迟任务集合 delayedTasks 和新增任务集合 newTasks
2. 对于所有新增的额外需求点:
   a. 将其任务ID添加到 newTasks
3. 对于每个车辆的静态阶段路径:
   a. 考虑高峰期影响，重新计算完成时间
   b. 找出由于高峰期影响，完成时间超过 staticMaxTime 的任务
   c. 将这些任务添加到 delayedTasks
4. 返回 delayedTasks 和 newTasks
````

### 5.2 动态阶段遗传算法

动态阶段遗传算法在静态最优解的基础上，允许延迟任务和新增任务分配给任意车辆。

````
算法：动态阶段遗传算法（dynamic_genetic.cpp）
输入：问题实例 problem，静态路径 staticPaths，延迟任务 delayedTasks，
     新任务 newTasks，种群大小 populationSize，迭代代数 generations，
     变异率 mutationRate，时间权重 timeWeight
输出：任务分配方案 assignments

1. 合并延迟任务和新任务到灵活任务集合 flexibleTasks
2. 收集所有任务点 allTaskIds
3. 创建静态任务到车辆的映射 staticTaskToVehicle
4. 创建配送中心到车辆的映射 centerToVehicles

5. 初始化种群 population，每个个体包含:
   a. 对于静态阶段未延迟的任务，保持原车辆分配
   b. 对于灵活任务集合中的任务，随机分配给任意车辆

6. 对每一代 gen from 0 to generations-1 执行:
   a. 创建新种群 newPopulation
   b. 计算每个个体的适应度
   c. 选择操作：选择较好的个体进入下一代
   d. 交叉操作：随机选择交叉点，交换部分基因
   e. 变异操作：
      i. 对于静态阶段未延迟的任务，只在同一配送中心的车辆间变异
      ii. 对于灵活任务集合中的任务，可分配给任意车辆
   f. 更新种群 population

7. 选择最优个体，构建最终分配结果 assignments
8. 返回 assignments
````

### 5.3 动态阶段路径优化

动态阶段路径优化考虑额外需求点的时间约束和高峰期的影响，同时实现车机协同。

#### 5.3.1 普通车辆动态路径优化

````
算法：普通车辆动态路径优化（Dynamic_OptimizePathForVehicle函数）
输入：分配给车辆的任务ID列表 assignedTaskIds，任务点集合 tasks，车辆 vehicle，问题实例 problem
输出：优化后的路径 path 和时间表 times

1. 若 assignedTaskIds 为空，返回 [[vehicle.centerId, vehicle.centerId], [0.0, 0.0]]
2. 初始化路径 path，添加配送中心ID，初始化时间表 times，添加初始时间0
3. 初始化已访问状态数组 visited，全部标记为未访问
4. 从配送中心开始，currentPos ← vehicle.centerId，currentTime ← 0.0
5. 当仍有未访问的任务点时，执行:
   a. 检查是否只剩下额外需求点未访问:
      i. 如果是，找出最早的额外需求点
      ii. 如果需要等待该额外需求点的到达时间，则更新当前时间
   b. 初始化 minDistance ← ∞
   c. 对每个未访问的任务点 i 执行:
      i. 计算到任务点的距离 distance
      ii. 考虑高峰期影响，计算实际行驶时间
      iii. 对于额外需求点，检查当前时间+行驶时间是否晚于其到达时间
      iv. 如果满足时间约束且 distance < minDistance，更新 minDistance 和下一个任务点
   d. 如果找到下一个任务点:
      i. 更新路径和已访问状态
      ii. 考虑高峰期影响，计算并更新当前时间
      iii. 记录到达时间
   e. 否则结束循环
6. 返回配送中心，更新路径和时间
7. 返回 [path, times]
````

#### 5.3.2 车机协同无人机路径优化

````
算法：车机协同无人机路径优化（optimizeDronePathWithVehicles函数）
输入：分配给无人机的任务ID列表 taskIds，任务点集合 tasks，无人机 drone，
     问题实例 problem，任务访问信息 taskVisitInfo
输出：优化后的路径 path 和时间表 times

1. 若 taskIds 为空，返回 [[drone.centerId, drone.centerId], [0.0, 0.0]]
2. 初始化路径 path，添加配送中心ID，初始化时间表 times，添加初始时间0
3. 初始化已访问状态数组 visited，全部标记为未访问
4. 从配送中心开始，currentPos ← drone.centerId，currentTime ← 0.0
5. 初始化电量 currentBattery ← drone.maxfuel，载重 currentLoad ← 0，过程最大载重 maxProcessLoad ← 0
6. 当仍有未访问的任务点时，执行:
   a. 如果只剩额外需求点未访问且需要等待:
      i. 等待直到最早额外需求点的到达时间
   b. 初始化 minDistance ← ∞
   c. 对每个未访问的任务点 i 执行:
      i. 计算到任务点的距离 distance
      ii. 检查电量约束：确保有足够电量到达该点并返回某个可充电点
      iii. 检查载重约束
      iv. 检查额外需求点时间约束
      v. 如果所有约束满足且 distance < minDistance，更新 minDistance 和下一个任务点
   d. 如果找到下一个任务点:
      i. 更新路径、时间、电量和载重
      ii. 更新已访问状态
   e. 否则:
      i. 找到可返回的点（配送中心或车辆访问点）
      ii. 计算返回时间（可能需要等待车辆到达）
      iii. 充电和卸货
7. 返回 [path, times]
````

## 6. 系统特点与创新

本系统的主要算法特点与创新点包括：

1. **分层优化策略**：将问题分为静态和动态两个阶段，静态阶段处理确定性需求，动态阶段处理变化需求，提高解决方案的稳定性和灵活性。
2. **多约束路径规划**：无人机路径规划同时考虑电量、载重和时间约束，保证路径的可行性。
3. **车机协同机制**：无人机可以与车辆协同工作，不仅可以返回配送中心，还可以在车辆访问点汇合，扩大配送范围。
4. **动态任务识别**：系统能够识别因交通拥堵导致的延迟任务和新增的额外需求点，并进行重新调度。
5. **时间敏感处理**：对于额外需求点的特定到达时间约束，系统通过等待机制确保在正确的时间访问。
6. **高峰期影响建模**：通过速度调整系数模拟高峰期交通拥堵对配送时间的影响，使规划更符合实际情况。

## 7. 算法复杂度分析

### 时间复杂度

- 初始任务分配：O(m×n)，其中m为配送中心数量，n为任务点数量
- 静态阶段遗传算法：O(g×p×n²)，其中g为代数，p为种群大小，n为任务点数量
- 路径优化：O(n²)，其中n为分配给单个车辆的任务点数量
- 动态阶段遗传算法：O(g×p×n²)，其中g为代数，p为种群大小，n为任务点总数
- 车机协同路径规划：O(v×n²)，其中v为车辆数量，n为任务点数量

### 空间复杂度

- 任务分配：O(m+n)
- 遗传算法：O(p×n)，其中p为种群大小，n为任务点数量
- 路径规划：O(n)
- 整体系统：O(v×n)，其中v为车辆数量，n为任务点数量
