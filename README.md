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
