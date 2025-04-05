# 第6章作业

## 1.作业说明

* 运行环境：ROS2 Humble
* 代码详见`src`目录，修改较大，将原始代码移植到ROS2平台中，主要修改的代码在`mpc_car`包中的`mpc_car.hpp`中。

完成内容

* 实现基本MPC功能。
* 实现带有时间延迟的MPC功能。

## 2.作业运行结果

运行命令

```bash
ros2 launch mpc_car simulation.launch.py
```

随机在地图上选点，得到rviz可视化运行结果如下图：

![](result.png)

