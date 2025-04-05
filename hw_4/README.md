# 第4章作业

## 1.作业说明

* 运行环境：ROS2 Humble
* 代码详见`src`目录，修改较大，将原始代码移植到ROS2平台中，主要修改的代码在`grid_path_searcher`包中的`hw_tool.hpp/cpp`, `demo2_node.hpp/cpp` 中。

主要实现内容：

* 在`demo2_node.cpp` 中，实现了前向运动积分。
* 在`hw_tool.cpp`中，基于ceres solver实现了`OBVP`问题的数值近似求解。

## 2.作业运行结果

运行命令

```bash
ros2 launch grid_path_searcher demo2.launch.py
```

随机在地图上选点，得到rviz可视化运行结果如下图：

![](result.png)



## 3.OBVP求解

详见`hw_tool.cpp`中

定义优化问题：

```c++
struct MyFunc {
  MyFunc(Eigen::Vector3d &p0, Eigen::Vector3d &pf, Eigen::Vector3d &v0, Eigen::Vector3d &vf)
      : p0_(p0), pf_(pf), v0_(v0), vf_(vf) {}

  template <typename T>
  bool operator()(const T* const x,
                  T* residuals) const {
    auto t = x[0];
    auto t2 = t*t;
    auto t3 = t2*t;
    auto dp = pf_ - v0_ * t - p0_;
    auto dv = vf_ - v0_;
    auto alpha = -12.0 / t3 * dp + 6.0 / t2 * dv;
    auto beta =  6.0 / t2 *  dp - 2.0 / t * dv;
    auto J = t;
    for(int i = 0; i<3; i++) {
        J += alpha(i)*alpha(i)*t3 + alpha(i)*beta(i)*t2 + beta(i)*beta(i)*t;
    }
    residuals[0] = J;
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
  static ceres::CostFunction* Create(Eigen::Vector3d &p0, Eigen::Vector3d &pf, Eigen::Vector3d &v0, Eigen::Vector3d &vf) {
    return (new ceres::AutoDiffCostFunction<MyFunc, 1, 1>(new MyFunc(p0, pf, v0, vf)));
  }

  Eigen::Vector3d p0_;
  Eigen::Vector3d pf_;
  Eigen::Vector3d v0_;
  Eigen::Vector3d vf_;
};

```

求解优化问题：

```c++
    // Build the problem.
    ceres::Problem problem;
    ceres::CostFunction* cost_function = MyFunc::Create(_start_position, _start_velocity, _target_position, target_velocity);
    problem.AddResidualBlock(cost_function, nullptr, &x);
    problem.SetParameterLowerBound(&x, 0, 0.001);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
```

补充说明：

* 我们的问题是最小化代价$J=\int^T_0(1+a^2_x+a^2_y+a^2_z)dt$，从`J`的定义中可以发现$J>0$，因此等价于最小化$J^2$ ，因此可以利用Ceres进行优化求解，同时设定$T>0.001$，避免得到无意义的负值。

