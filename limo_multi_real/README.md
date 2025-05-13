# ROS编队小车使用说明

## 功能介绍

本项目实现了基于ROS的多机器人编队系统，支持主从车编队跟随功能。主要特点：

1. **不依赖navigation包**：通过自定义TF转换系统实现从车跟随功能
2. **初始位置校正**：从车启动时自动校正初始位置
3. **车头方向偏差修复**：解决主从车车头方向偏差问题
4. **避障功能**：支持从车避障功能

## 主要改进

1. 添加了`tf_broadcaster.py`：创建完整的TF转换树，不依赖navigation包
2. 添加了`initial_pose_calibrator.py`：实现从车初始位置校正
3. 添加了`heading_corrector.py`：修复车头方向偏差问题
4. 修改了`limo_slave_no_map.launch`：整合所有新功能

## 使用方法

### 主车启动

```bash
# 启动主车
roslaunch limo_multi limo_master.launch
```

### 从车启动

```bash
# 启动从车（不依赖navigation）
roslaunch limo_multi limo_slave_no_map.launch
```

## 参数配置

在`limo_slave_no_map.launch`中可以配置以下参数：

- `slave_x`和`slave_y`：从车相对于主车的期望位置
- `multi_mode`：编队模式选择（1：主车自转，从车围绕主车运动；2：主车自转，从车原地自转）
- `avoidance`：是否开启避障功能
- `max_vel_x`和`min_vel_x`：从车最大和最小线速度限制
- `max_vel_theta`和`min_vel_theta`：从车最大和最小角速度限制

## 技术说明

### TF转换系统

系统通过以下TF转换实现从车跟随功能：

1. 主车odom到主车base_link的转换
2. 主车base_link到从车期望位置的转换
3. 从车odom到从车base_link的转换
4. map到主车odom和从车odom的转换

### 初始位置校正

从车启动时，系统会：

1. 获取主车当前位置和朝向
2. 计算从车的理想位置
3. 比较从车当前位置与理想位置的差距
4. 如果差距超过阈值，进行位置校正

### 车头方向校正

系统通过以下步骤修复车头方向偏差：

1. 计算主从车头方向差异
2. 根据差异调整从车角速度
3. 应用校正后的速度命令

### 避障功能

避障功能通过以下步骤实现：

1. 检测障碍物距离和方向
2. 根据障碍物情况调整速度和方向
3. 在安全距离内减速，在危险距离内停止并避让

## 故障排除

如果从车跟随不正常，请检查：

1. 主从车之间的通信是否正常
2. TF转换是否正确建立
3. 初始位置校正是否成功
4. 车头方向校正是否生效

可以使用以下命令查看TF树：

```bash
rosrun tf view_frames
```

或者使用以下命令查看特定的TF转换：

```bash
rosrun tf tf_echo <source_frame> <target_frame>
```
