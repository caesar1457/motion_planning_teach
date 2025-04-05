# 第5章作业

## 1.作业说明

* 运行环境：ROS2 Humble
* 代码详见`src`目录，修改较大，将原始代码移植到ROS2平台中，主要修改的代码在`trajectory_optimization`包中的`click_gen.cpp`中。

主要实现内容：

* 在`click_gen.cpp` 中，实现了M矩阵的构造，由于demo中问题规模较小，直接采用M矩阵求逆，从而计算系数矩阵c。

## 2.作业运行结果

运行命令

```bash
ros2 launch trajectory_optimization click_gen.launch.py
```

随机在地图上选点，得到rviz可视化运行结果如下图：

![](result.png)

