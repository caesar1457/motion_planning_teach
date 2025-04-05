# 第 1 章作业：路径搜索模块（基于 ROS2）

本作业已经迁移至 ROS2 Humble 环境，支持标准 `colcon` 构建与 `launch.py` 启动方式。

---

## 项目结构

```
hw_1/
├── grid_path_searcher/         # 路径搜索主程序包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/demo.launch.py
│   ├── src/
│   └── include/
├── rviz_plugins/               # RViz 插件（目标点选择工具）
└── waypoint_generator/         # 路径采样与轨迹生成
```

---

## 一、准备工作空间

### 1. 创建 ROS2 工作空间

```bash
mkdir -p ~/motion_planning_ws/src
cd ~/motion_planning_ws/src
```

### 2. 下载/放置本作业代码

将 `hw_1/` 文件夹拷贝或下载到 `src/` 目录下：

```bash
cd ~/motion_planning_ws/src
# 如果是压缩包，请先解压
# unzip hw_1.zip
```

---

## 二、运行环境

- 操作系统：Ubuntu 22.04
- ROS 版本：ROS 2 Humble
- 构建工具：colcon
- 可视化工具：RViz2

确保已正确安装 ROS 2 Humble，并执行 ROS2 环境配置：

```bash
source /opt/ros/humble/setup.bash
```

---

## 三、构建项目

进入工作空间根目录并构建：

```bash
cd ~/motion_planning_ws
colcon build --packages-select grid_path_searcher
```

构建成功后，配置环境变量：

```bash
source install/setup.bash
```

---

## 四、运行路径搜索系统

启动主路径解算节点：

```bash
ros2 launch grid_path_searcher demo.launch.py
```

启动后将自动打开 RViz，展示场景、路径及起终点。

---

## 五、运行效果（截图）

运行成功后，你将在 RViz 中看到如下内容：

- 标准的标量地图环境
- 起点、终点标记（可手动设置）
- A* 搜索结果路径（蓝色线段）
- 控制台打印路径信息和搜索状态

> ✅ 同学请截取运行截图并随作业一同提交。

![](result.png)

---

## 六、常见问题

### Q1: 找不到包 `grid_path_searcher`
请确认是否运行了环境声明命令：

```bash
source install/setup.bash
```

### Q2: 构建报错找不到 `catkin_pkg`
请使用系统 Python，不要在 Anaconda 环境下构建：

```bash
which python3
# 应为：/usr/bin/python3
```

---

如有任何问题，请联系助教或参考终端输出进行排查。





