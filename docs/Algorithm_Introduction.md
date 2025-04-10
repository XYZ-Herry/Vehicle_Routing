# 路径规划系统算法介绍

## 1. 系统概述

本系统实现了一种复杂的车辆和无人机混合配送路径规划解决方案，分为静态阶段和动态阶段。静态阶段处理已知的任务需求，而动态阶段则处理额外需求点和由于交通状况变化导致的延迟任务。系统采用遗传算法进行任务分配优化，使用改进的最近邻算法进行路径规划，并实现了车辆与无人机的协同工作机制。

## 2. 系统算法流程

整个系统算法流程如下：

```
算法：路径规划系统主流程
输入：配送中心、车辆信息、任务点信息
输出：优化后的配送路径和时间

1. 数据预处理：读入并处理所有数据（common.cpp）
2. 任务初始分配：每个任务点分配到距离最近的配送中心（assignTasksToCenters函数）
3. 静态阶段遗传算法：按配送中心分别进行遗传算法，为各中心的各车辆/无人机分配任务点（static_genetic.cpp）
4. 静态阶段路径优化：优化各车辆/无人机的路径并计算完成时间（optimizePathForVehicle函数）
5. 计算静态阶段最晚完成时间T（main.cpp里面）
6. 动态阶段任务识别：收集超过时间T的延迟任务和新增任务（solver.cpp里面的identifyTasksForRescheduling函数）
7. 动态阶段遗传算法：在静态最优解基础上重新分配任务点（dynamic_genetic.cpp）
8. 动态阶段路径优化：考虑车机协同优化所有路径（Dynamic_OptimizePathForVehicle和optimizeDronePathWithVehicles函数）
9. 返回并输出最终优化的路径和时间表（main.cpp里的Print_DeliveryResults函数）
10. 验证返回的静态阶段和动态阶段最终路径的合法性（path_validator.cpp）
```

## 3. 初始任务分配算法

初始任务分配将每个任务点分配给距离最近的配送中心，确定任务点的管辖范围。

```
算法 3.1: assignTasksToCenters函数
输入: 配送中心集合 centers，任务点集合 tasks
输出: 各配送中心的任务点映射 centerToTasks

1   初始化 centerToTasks 为空映射                /* 准备存储每个中心负责的任务 */
2   for 每个任务点 task in tasks do
3     minDistance ← ∞                          /* 初始化最小距离为无穷大 */
4     nearestCenter ← -1                       /* 初始化最近中心为未找到 */
5     for 每个配送中心 center in centers do
6       distance ← 计算任务点到配送中心的实际距离    /* 使用距离矩阵或直接计算 */
7       if distance < minDistance then
8         minDistance ← distance
9         nearestCenter ← center.id
10      end if
11    end for
12    centerToTasks[nearestCenter].push(task.id)  /* 添加任务到对应中心 */
13    task.centerId ← nearestCenter               /* 更新任务点的所属中心 */
14  end for
15  return centerToTasks                          /* 返回分配结果 */
```

## 4. 静态阶段优化

### 4.1 静态阶段遗传算法

静态阶段采用按配送中心划分的遗传算法，每个配送中心的车辆只负责本辖区内的任务点。

```
算法 4.1: staticGeneticAlgorithm函数
输入: 问题实例 problem，种群大小 populationSize，迭代代数 generations，
     变异率 mutationRate，时间权重 timeWeight
输出: 任务分配方案 assignments

1   assignments ← ∅                           /* 初始化空的分配方案 */
2   for 每个配送中心 centerId in problem.centerIds do
3     if 该配送中心没有任务点 then               /* 跳过没有任务的中心 */
4       continue
5     end if
6     centerVehicleIds ← 找出属于该中心的所有车辆  /* 收集该中心的车辆 */
7     centerTaskIds ← 获取该中心负责的所有任务点   /* 收集该中心的任务 */
8     population ← 初始化种群                   /* 随机生成初始分配方案 */
9     for gen ← 0 to generations-1 do         /* 迭代优化 */
10      newPopulation ← ∅
11      计算每个个体的适应度                      /* 评估每个分配方案 */
12      从population中选择较好的个体              /* 保留优秀解 */
13      执行交叉操作，产生新个体                  /* 组合优秀解特征 */
14      以mutationRate的概率执行变异操作          /* 随机变异避免局部最优 */
15      更新种群 population                    /* 保留可行解 */
16    end for
17    将最优个体的分配方案添加到 assignments       /* 保存该中心的最佳方案 */
18  end for
19  return assignments                        /* 返回所有中心的分配结果 */
```

### 4.2 静态阶段路径优化

根据分配结果，分别对普通车辆和无人机进行路径优化。

#### 4.2.1 普通车辆路径优化

普通车辆采用简单的贪心策略进行路径优化。

```
算法 4.2.1: optimizePathForVehicle函数(普通车辆部分)
输入: 分配给车辆的任务ID列表 assignedTaskIds，任务点集合 tasks，车辆 vehicle，问题实例 problem
输出: 优化后的路径 path

1   if assignedTaskIds = ∅ then                /* 无任务情况处理 */
2     return [vehicle.centerId, vehicle.centerId]  /* 返回空路径 */
3   end if
4   path ← [vehicle.centerId]                  /* 初始化路径，从配送中心开始 */
5   visited ← [false,...,false]                /* 标记所有任务为未访问 */
6   currentPos ← vehicle.centerId              /* 当前位置为配送中心 */
7   while 存在未访问的任务点 do                  /* 主循环：访问所有任务点 */
8     minDistance ← ∞                         /* 初始化最小距离 */
9     nextId ← -1                             /* 初始化下一个任务点ID */
10    for 每个未访问的任务点 i do
11      distance ← getDistance(currentPos, assignedTaskIds[i], problem, false)
12      if distance < minDistance then         /* 找出最近的任务点 */
13        minDistance ← distance
14        nextId ← assignedTaskIds[i]
15      end if
16    end for
17    if nextId ≠ -1 then                      /* 如果找到下一个任务点 */
18      将nextId标记为已访问
19      path ← path ∪ {nextId}                 /* 添加到路径 */
20      currentPos ← nextId                    /* 更新当前位置 */
21    else
22      break                                  /* 无法找到下一个点，结束循环 */
23    end if
24  end while
25  path ← path ∪ {vehicle.centerId}           /* 返回配送中心 */
26  return path
```

#### 4.2.2 无人机路径优化

无人机路径优化需要考虑电量和载重约束。

```
算法 4.2.2: optimizePathForVehicle函数(无人机部分)
输入: 分配给无人机的任务ID列表 assignedTaskIds，任务点集合 tasks，无人机 drone，问题实例 problem
输出: 优化后的路径 path

1   if assignedTaskIds = ∅ then                /* 无任务情况处理 */
2     return [drone.centerId, drone.centerId]  /* 返回空路径 */
3   end if
4   path ← [drone.centerId]                    /* 初始化路径，从配送中心开始 */
5   visited ← [false,...,false]                /* 标记所有任务为未访问 */
6   currentPos ← drone.centerId                /* 当前位置为配送中心 */
7   currentBattery ← drone.maxfuel             /* 初始电量满格 */
8   currentLoad ← 0                            /* 初始载重为0 */
9   maxProcessLoad ← 0                         /* 记录过程最大载重 */
10  while 存在未访问的任务点 do                  /* 主循环：访问所有任务点 */
11    minDistance ← ∞                          /* 初始化最小距离 */
12    nextId ← -1                              /* 初始化下一个任务点ID */
13    for 每个未访问的任务点 i do
14      taskId ← assignedTaskIds[i]
15      distance ← getDistance(currentPos, taskId, problem, true)
16      returnDistance ← getDistance(taskId, drone.centerId, problem, true)
17      batteryNeeded ← distance / drone.speed  /* 计算所需电量 */
18      batteryToReturn ← returnDistance / drone.speed  /* 返回所需电量 */
19      /* 检查电量和载重约束 */
20      if batteryNeeded + batteryToReturn <= currentBattery and 满足载重约束 then
21        if distance < minDistance then
22          minDistance ← distance
23          nextId ← taskId
24        end if
25      end if
26    end for
27    if nextId ≠ -1 then                       /* 如果找到下一个任务点 */
28      将nextId标记为已访问
29      path ← path ∪ {nextId}                  /* 添加到路径 */
30      currentBattery ← currentBattery - minDistance/drone.speed  /* 更新电量 */
31      更新载重信息(currentLoad, maxProcessLoad, nextId)
32      currentPos ← nextId                     /* 更新当前位置 */
33    else                                      /* 无法继续访问任务点 */
34      path ← path ∪ {drone.centerId}          /* 返回配送中心 */
35      currentBattery ← drone.maxfuel          /* 充电 */
36      currentLoad ← 0                         /* 卸货 */
37      maxProcessLoad ← 0                      /* 重置最大载重 */
38      currentPos ← drone.centerId             /* 更新当前位置 */
39    end if
40  end while
41  return path
```

## 5. 动态阶段优化

### 5.1 动态阶段任务识别

动态阶段首先识别需要重新调度的任务，包括由于交通状况变化导致的延迟任务和新增的额外需求任务。

```
算法 5.1: identifyTasksForRescheduling函数
输入: 问题实例 problem，静态路径 staticPaths，静态阶段最晚完成时间 staticMaxTime
输出: 延迟任务集合 delayedTasks，新增任务集合 newTasks

1   delayedTasks ← ∅                           /* 初始化延迟任务集合 */
2   newTasks ← ∅                               /* 初始化新增任务集合 */
3   for 每个额外需求点 task in problem.extraTasks do
4     newTasks ← newTasks ∪ {task.id}          /* 收集所有新增的额外需求点 */
5   end for
6   for 每个车辆的静态阶段路径 (vehicleId, path, times) in staticPaths do
7     recalculatedTimes ← 考虑高峰期重新计算完成时间  /* 评估高峰期影响 */
8     for i ← 0 to path.size()-1 do
9       if not 是配送中心(path[i]) and recalculatedTimes[i] > staticMaxTime then
10        delayedTasks ← delayedTasks ∪ {path[i]}  /* 收集延迟任务 */
11      end if
12    end for
13  end for
14  return delayedTasks, newTasks              /* 返回需要重新调度的任务 */
```

### 5.2 动态阶段遗传算法

动态阶段遗传算法在静态最优解的基础上，允许延迟任务和新增任务分配给任意车辆。

```
算法 5.2: dynamicGeneticAlgorithm函数
输入: 问题实例 problem，静态路径 staticPaths，延迟任务 delayedTasks，
     新任务 newTasks，种群大小 populationSize，迭代代数 generations，
     变异率 mutationRate，时间权重 timeWeight
输出: 任务分配方案 assignments

1   flexibleTasks ← delayedTasks ∪ newTasks    /* 合并需要重新分配的任务 */
2   allTaskIds ← 收集所有任务点ID               /* 包括静态阶段未延迟的任务 */
3   staticTaskToVehicle ← 创建任务到车辆的映射   /* 记录静态阶段的分配情况 */
4   centerToVehicles ← 创建中心到车辆的映射      /* 记录每个中心可用的车辆 */

5   population ← 初始化种群，每个个体包含:       /* 创建初始种群 */
6     对于静态阶段未延迟的任务，保持原车辆分配
7     对于flexibleTasks中的任务，随机分配给任意车辆

8   for gen ← 0 to generations-1 do           /* 迭代优化 */
9     newPopulation ← ∅
10    计算每个个体的适应度                       /* 评估分配方案 */
11    选择操作：选择较好的个体                    /* 保留优秀解 */
12    交叉操作：随机交换部分任务分配              /* 组合优秀解特征 */
13    变异操作:                                /* 随机变异避免局部最优 */
14      对于静态阶段未延迟的任务，只在同一配送中心的车辆间变异
15      对于flexibleTasks中的任务，可分配给任意车辆
16    更新种群 population
17  end for

18  选择最优个体，构建最终分配结果 assignments    /* 选择最佳方案 */
19  return assignments                        /* 返回任务分配方案 */
```

### 5.3 动态阶段路径优化

动态阶段路径优化考虑额外需求点的时间约束和高峰期的影响，同时实现车机协同。

#### 5.3.1 普通车辆动态路径优化

```
算法 5.3.1: Dynamic_OptimizePathForVehicle函数
输入: 分配给车辆的任务ID列表 assignedTaskIds，任务点集合 tasks，车辆 vehicle，问题实例 problem
输出: 路径 path 和时间表 times

1   if assignedTaskIds = ∅ then
2     return [[vehicle.centerId, vehicle.centerId], [0.0, 0.0]]
3   end if

4   path ← [vehicle.centerId]
5   times ← [0.0]
6   visited ← [false,...,false]
7   currentPos ← vehicle.centerId
8   currentTime ← 0.0

9   while 存在未访问的任务点 do
10    minDistance ← ∞
11    nextIndex ← -1

12    for i ← 0 to assignedTaskIds.size()-1 do
13      if not visited[i] then
14        taskId ← assignedTaskIds[i]
15        distance ← getDistance(currentPos, taskId, problem, false)
16        timeToTask ← calculateTimeNeeded(currentPos, taskId, currentTime, vehicle, problem, true, false)
        
17        if 是额外需求点 and currentTime + timeToTask < 规定到达时间 then
18          continue
19        end if
        
20        if distance < minDistance then
21          minDistance ← distance
22          nextIndex ← i
23        end if
24      end if
25    end for

26    if nextIndex ≠ -1 then
27      visited[nextIndex] ← true
28      nextId ← assignedTaskIds[nextIndex]
29      path ← path ∪ {nextId}
30      timeNeeded ← calculateTimeNeeded(currentPos, nextId, currentTime, vehicle, problem, true, false)
31      currentTime ← currentTime + timeNeeded
32      times ← times ∪ {currentTime}
33      currentPos ← nextId
34    else
35      break
36    end if
37  end while

38  if currentPos ≠ vehicle.centerId then
39    path ← path ∪ {vehicle.centerId}
40    timeNeeded ← calculateTimeNeeded(currentPos, vehicle.centerId, currentTime, vehicle, problem, true, false)
41    currentTime ← currentTime + timeNeeded
42    times ← times ∪ {currentTime}
43  end if

44  return [path, times]
```

#### 5.3.2 车机协同无人机路径优化

```
算法 5.3.2: optimizeDronePathWithVehicles函数
输入: 分配给无人机的任务ID列表 taskIds，任务点集合 tasks，无人机 drone，
     问题实例 problem，任务访问信息 taskVisitInfo
输出: 路径 path 和时间表 times

1   if taskIds = ∅ then
2     return [[drone.centerId, drone.centerId], [0.0, 0.0]]
3   end if

4   path ← [drone.centerId]
5   times ← [0.0]
6   visited ← [false,...,false]
7   currentPos ← drone.centerId
8   currentTime ← 0.0
9   currentBattery ← drone.maxfuel
10  currentLoad ← 0.0

11  while 存在未访问的任务点 do
12    minDistance ← ∞
13    nextIndex ← -1

    /* 寻找满足约束的下一个任务点 */
14    for i ← 0 to taskIds.size()-1 do
15      if not visited[i] then
16        taskId ← taskIds[i]
17        distance ← getDistance(currentPos, taskId, problem, true)
18        batteryNeeded ← distance / drone.speed
        
19        if 满足电量、载重和时间约束 then
20          if distance < minDistance then
21            minDistance ← distance
22            nextIndex ← i
23          end if
24        end if
25      end if
26    end for

27    if nextIndex ≠ -1 then
28      visited[nextIndex] ← true
29      nextId ← taskIds[nextIndex]
30      path ← path ∪ {nextId}
31      flyingTime ← minDistance / drone.speed
32      currentTime ← currentTime + flyingTime
33      times ← times ∪ {currentTime}
34      currentBattery ← currentBattery - flyingTime
35      更新载重(currentLoad, nextId, problem)
36      currentPos ← nextId
37    else
38      /* 寻找可返回的点（配送中心或车辆协同点）*/
39      returnPoint ← 找到合适的返回点
40      path ← path ∪ {returnPoint}
41      /* 计算返回时间并可能等待车辆 */
42      更新时间(currentTime, times)
43      /* 充电和卸货 */
44      currentBattery ← drone.maxfuel
45      currentLoad ← 0.0
46      currentPos ← returnPoint
47    end if
48  end while

49  return [path, times]
```

#### 5.3.3 高峰期速度系数计算

```
算法 5.3.3: getSpeedFactor函数
输入: 当前时间 currentTime，起点ID fromId，终点ID toId，问题实例 problem
输出: 速度系数 speedFactor

1   isMorningPeak ← (currentTime ≥ MORNING_PEAK_START and currentTime ≤ MORNING_PEAK_END)
2   isEveningPeak ← (currentTime ≥ EVENING_PEAK_START and currentTime ≤ EVENING_PEAK_END)
    
3   if not (isMorningPeak or isEveningPeak) then
4     return 1.0  /* 非高峰期 */
5   end if
    
6   if problem.network.peakFactors存在fromId到toId的记录 then
7     if isMorningPeak then
8       return 早高峰系数
9     else
10      return 晚高峰系数
11    end if
12  end if
    
    /* 使用默认系数 */
13  if isMorningPeak then
14    return DEFAULT_MORNING_PEAK_FACTOR
15  else
16    return DEFAULT_EVENING_PEAK_FACTOR
17  end if
```

#### 5.3.4 考虑高峰期的时间计算

```
算法 5.3.4: calculateTimeNeeded函数
输入: 起点ID currentId，终点ID destId，当前时间 currentTime，车辆 vehicle，
     问题实例 problem，是否考虑高峰期 considerTraffic，是否为无人机 isDrone
输出: 所需时间 timeNeeded

1   distance ← getDistance(currentId, destId, problem, isDrone)
    
2   if not considerTraffic or isDrone then
3     return distance / vehicle.speed  /* 不考虑高峰期或无人机直接计算 */
4   end if
    
5   remainingDistance ← distance
6   totalTime ← 0.0
7   travelTime ← currentTime
    
8   while remainingDistance > 0.0001 do
9     isPeakHour ← checkIfPeakHour(travelTime)
10    speedFactor ← 1.0
11    if isPeakHour then
12      speedFactor ← getSpeedFactor(travelTime, currentId, destId, problem)
13    end if
14    currentSpeed ← vehicle.speed * speedFactor
    
15    timeToNextPhase ← 计算到下一个时间段的时间
16    distanceCanTravel ← currentSpeed * timeToNextPhase
    
17    if distanceCanTravel ≥ remainingDistance then
18      totalTime ← totalTime + remainingDistance / currentSpeed
19      remainingDistance ← 0
20    else
21      totalTime ← totalTime + timeToNextPhase
22      remainingDistance ← remainingDistance - distanceCanTravel
23      travelTime ← travelTime + timeToNextPhase
24      处理可能的跨天情况
25    end if
26  end while
    
27  return totalTime
```

## 6. 系统特点与创新

本系统的主要算法特点与创新点包括：

1. **分层优化策略**：将问题分为静态和动态两个阶段，静态阶段处理确定性需求，动态阶段处理变化需求，提高解决方案的稳定性和灵活性。
2. **多约束路径规划**：无人机路径规划同时考虑电量、载重和时间约束，保证路径的可行性。
3. **车机协同机制**：无人机可以与车辆协同工作，不仅可以返回配送中心，还可以在车辆访问点汇合，扩大配送范围。
4. **动态任务识别**：系统能够识别因交通拥堵导致的延迟任务和新增的额外需求点，并进行重新调度。
5. **时间敏感处理**：对于额外需求点的特定到达时间约束，系统通过等待机制确保在正确的时间访问。
6. **高峰期影响建模**：通过速度调整系数模拟高峰期交通拥堵对配送时间的影响，使规划更符合实际情况。
