# 1 move_base 

主要实现对globalPlanner、localPlanner的管控，并获取到对应的planner_costmap_ros_、controller_costmap_ros_
在线程planThread，中执行makeplan

## 1 init 

1. 加载globalPlanner 插件、localPlanner插件、recoveryBehaviors插件
2. 初始化 MoveBaseActionServer as_ 
3. 初始化参数，具体参数查看move_base_params.yaml
4. 初始化planThread
5. 根据参数创建对应的globalPlanner、localPlanner、recoveryBehaviors插件,并开启costmap_2d
	globalPlanner:planner_,planner_costmap_ros_
	localPlanner:tc_,controller_costmap_ros_ 
	recoveryBehaviors:recovery_behaviors_

## 2 planThread

1. 初始化后进入循环中
2. 需要唤醒或未执行规划,等待条件变量的触发
3. 获取plan
3.1. 获取成功,修改move_base的state为CONTROLLING,latest_plan_ 为最新的plan
3.2. 获取失败,则重试,直到超时或超过最大获取次数,修改move_base的state为 CLEARING,recovery_trigger_ = PLANNING_R

## 3 executeCb

```c++
void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
```
1. 获取goal = move_base_goal.target_pose,开启global planner
2. 循环等待, executeCycle 判断是否完成
3. 循环判断，是否有新的goal preempt

## 4 executeCycle

```c++
bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
```

1. 通过global planner:planner_costmap_ros_ 获取当前机器人位置,作为feedback发出
2. 判断机器人是否在震荡，如果先前的 recovery_trigger_==OSCILLATION_R(即因为震荡问题进入recoveryBehaviors)
此时若离开了起始的震荡位置，则回复recovery_index_=0(即recoverBehavior重新设置为第一个)
3. localPlanner 传入global算出的path
4. 判断状态state_
4.1 如果是PLANNING:则等待新的plan
4.2 如果是CONTROLLING:判断是否抵达目标点;判断是否震荡超时;获取速度,如果获取不到且获取超时，则触发恢复动作
4.3 如果是CLEARING:
4.3.1 如果恢复动作还未做完,则执行恢复动作后再进入PLANNING状态
4.3.2 如果做完了，报错并终止本次goal

# 2 costmap_2d

## 1 init

1. 获取参数
2. 创建master costmap :layered_costmap_,加载插件,通过它管理各层地图
3. 订阅footprint，当话题上收到footprint时，回调函数会将接收到的footprint根据参数footprint_padding_的值进行膨胀，得到“膨胀”后的padded_footprint_，传递给各级地图

# 3 costmap plugin

# 4 globalPlanner

在move_base中，只会调用initialize和makePlan

## initialize

1. params:

frame_id_:全局坐标系，与global_costmap一致

convert_offset_:"old_navfn_behavior":true为0.0；false为0.5

p_calc_:潜在值，根据"use_quadratic"判断是否使用QuadraticCalculator

planner_:全局规划器,根据"use_dijkstra"使用算法

path_maker_:轨迹回溯规划器,根据"use_grid_path"判断是否根据网格来回溯

## makePlan

1. 判断start，goal的frame_id是否是costmap的全局坐标系

2. 得到以下的变量:

wx,wy:世界坐标的位置

start_x_i,start_y_i,goal_x_i,goal_y_i:起点终点的costs的unsigned int坐标

start_x,start_y,goal_x,goal_y:起点终点的costs的double坐标

3. 清除costs起点和终点的cost给costmap描边

4. 使用dijkstra,或astar,绘制potential地图

5. 在potential图中继续描绘以goal为中心描绘5*5的地方

6. 发布potential地图

5. 使用path_maker_计算出路径

--------------------------

### Expander类

### 1 AStar

变量:

costs:从layered_costmap_获取到的costs

queue_:保存还未计算potential的点

potential:大小为global_costmap的大小，初始化时全部设为1e+10，起点设为0

lethal_cost_(253)致命cost，neutral_cost_(50)中立cost, factor_(3.0)

算法：

1. 将整个potential设为1e+10,通过距离和costs以及potential,去计算queue_起点周围4个点的值,并插入queue_
取最小，并删除最小 然后找最小值周围的四个点,继续赋值，直到找到goal

2. potential计算，通过当前点的四周围的potential值以及cost计算出当前点的potential值

### 2 dijkstra

--------------------------

### Traceback类

### 1 gradient_path

变量:

pathStep_(0.5):

stc:当前点在potential中的index
stcpx:当前点上方在potential中的index
stcnx:当前点下方在potential中的index

gradx_,xgrady_:x梯度值 ,y梯度值,大小为地图大小


dx,dy:double和unsigned int坐标的差值

ns:地图面积

nx,ny:将stc转为实际位置

算法:

1. 将当前点nx ny放入path中

2. 判断路径是否震荡，或当前点周围的9个点是否未设值

2.1 ture:取当前点四周的8个点potential最低点作为stc,dx=dy=0;如果8个点都未设值，则退出

2.2 false: 计算当点，右点，下点，下右点的梯度

计算每个点到他四周的potential的梯度值

dx = 左点-右点的potential值
dy = 上点-下点的potential值

norm为dx dy的平方根

gradx_[n] = dx / norm , grady_[n] = dy / norm
x正数代表了向右，y正代表向下

```
potential
起点为0.0 终点为200.0
xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx 
xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx 
xxxxx.xxx xxxxx.xxx xxxxx.xxx 00177.246 00209.929 00259.929 00306.929 00353.929 xxxxx.xxx xxxxx.xxx xxxxx.xxx 
xxxxx.xxx 00050.000 00085.200 00127.246 00172.139 00218.553 00265.553 00312.553 xxxxx.xxx xxxxx.xxx xxxxx.xxx 
00050.000 00000.000 00050.000 00100.000 00150.000 00200.000 00247.000 00294.000 xxxxx.xxx xxxxx.xxx xxxxx.xxx 
xxxxx.xxx xxxxx.xxx xxxxx.xxx 00147.000 00181.636 00222.819 00265.877 00310.018 xxxxx.xxx xxxxx.xxx xxxxx.xxx 
xxxxx.xxx xxxxx.xxx xxxxx.xxx 00194.000 00220.537 00254.811 00293.172 00333.811 xxxxx.xxx xxxxx.xxx xxxxx.xxx 
xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx 
xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx xxxxx.xxx 
```

```
stcpx-1                stcpx                stcpx+1
      a------------------b------------------c   -
      |                  |                  |   |
      |                  |                  |   |  
      |                  |                  |   |
      |                  |                  |   |
      |                  |                  |  0.05(resolution)
      |                  |                  |   |  
      |                  |                  |   |
      |                  |                  |   |
 stc-1|                  |stc               |   |              
      d------------------e------------------f   -
      |                  |       |          | stc+1
      |                  |       |          |
      |                  |       |          |
      |                  |       dy         |
      |                  |       |          |
      |                  |       |          |
      |                  |       |          |
      |                  |---dx--* goal     |
      |                  |                  |
      g------------------h------------------i
stcnx-1                stcnx                stcnx+1

//双线性插值方法计算
//为什么用1.0-dx表示离stc的权重：当dx越小 说明stc占比越高
float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
float y = (1.0 - dy) * y1 + dy * y2; // interpolated y
```

```
float ss = pathStep_ / hypot(x, y);
dx += x * ss;
dy += y * ss;
```

x y为实际需要移动的方向的权值，通过pathStep_计算出dx和dy

如果dx，dy超过1则stc进行对应的加减




### 2 grid_path
--------------------------



# 5 localPlanner

