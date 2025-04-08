# 运动规划教学项目 ROS2 迁移总体说明

项目地址：[https://github.com/caesar1457/motion_planning_teach](https://github.com/caesar1457/motion_planning_teach)

## 一、项目概览

本项目将原基于 ROS1 的运动规划教学内容全面迁移至 ROS 2 Humble。迁移过程中针对旧版本存在的文档风格杂乱、构建流程复杂、启动方式繁琐等问题，进行了系统性的优化与重构，显著提升了教学适配性与维护效率。

### 存在问题（ROS1 旧版本）
- 作业说明风格不一致，结构混乱，阅读和操作体验较差。
- 学生需手动调整包结构与构建脚本（如 CMakeLists.txt），上手门槛较高。
- 启动依赖多个手动步骤，无法实现一键运行。

### ROS2 迁移后的核心改进
- 所有章节封装为**独立 ROS2 包**，彻底解耦，便于布置、测试与维护。
- 各章节作业支持**一键启动**，通过按钮或单行命令完成启动流程。
- 所有文档采用统一的**标准化 README 格式**，同步提供 PDF 版本，方便查阅与分发。
- 启动统一采用 `launch.py` 文件，参数集中于 YAML 配置，结构清晰。
- 支持 **Gazebo 仿真** 与 **RViz 可视化**，运行效果直观展示。

## 二、使用流程

### 环境要求
- 操作系统：Ubuntu 22.04
- ROS 版本：ROS 2 Humble
- 构建工具：colcon
- 各章节位于独立 ROS2 包中，互不依赖

### 编译方式
```bash
cd ~/ros2_ws
colcon build --packages-select xxx_chapter
source install/setup.bash
```

### 启动方式
```bash
ros2 launch xxx_chapter planner_launch.py
```

### 其他说明
- **第六章**涉及第三方库，已提供**完整编译脚本与详细流程说明**，确保可顺利构建。
- 本项目不再使用传统教学视频，转而采用**截图结合 PDF 文档**，逐步展示操作过程和执行结果，信息更集中、更易于跟进。

## 三、文档结构规范

各章节的 `README.md` 文件结构统一，主要包括：
- 本章学习目标
- 文件与目录结构说明
- 环境配置与构建流程
- 编译成功截图
- 作业说明与操作步骤
- 实验结果展示（含运行截图）
- 作者与维护人信息

## 四、系统迁移对比表

| 模块 | ROS1 版本 | ROS2 版本 |
|------|-----------|------------|
| 启动方式 | roslaunch xxx.launch | ros2 launch xxx_launch.py |
| 配置结构 | .launch / .yaml | launch.py + YAML |
| 消息通信 | rospy / roscpp | rclpy / rclcpp |
| 定时器机制 | rospy.Timer | rclpy.Timer |
| TF 处理 | tf | tf2_ros |
| 构建方式 | catkin_make | colcon build |
| 包组织方式 | 手动处理、结构不稳定 | 每章节独立 ROS2 包，结构清晰、可维护 |

## 五、关键章节优化说明

- **第一章**：作业流程显著简化，环境配置自动化，初学者可快速完成任务。
- **第三章**：新增 Informed RRT\* 路径规划模块，增强采样效率与搜索深度。
- **第六章**：引入第三方库，提供编译过程，显著降低配置难度。

## 六、总结与展望

本项目已顺利完成从 ROS1 向 ROS2 的整体迁移，系统性解决了原教学平台存在的兼容性差、配置复杂、结构混乱等核心问题。通过模块化设计、一键运行流程、统一文档体系等多项优化，有效提升了教学效率和使用体验。

本教学框架不仅便于学生快速上手，也为后续课程拓展、功能增强与路径规划算法实验提供了坚实基础。希望本项目在教学之外，也能作为 ROS2 项目的组织模板，为其他开发者与教学团队提供可借鉴的参考范式。

## 👥 Authors and Maintainers
_This README was written by the current maintainer based on the original project developed by the authors below._


<hr/>

<p align="right">
  <strong>Original Authors:</strong><br>
  Zhiye Zhao &lt;caesar1457@gmail.com&gt;<br><br>


  <strong>Current Maintainer:</strong><br>
  Zhiye Zhao &lt;caesar1457@gmail.com&gt; (2025–)
</p>