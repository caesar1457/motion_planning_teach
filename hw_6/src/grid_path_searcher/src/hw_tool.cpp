#include "grid_path_searcher/hw_tool.hpp"
#include "ceres/ceres.h"

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::getCost(Eigen::Vector3d &p0, Eigen::Vector3d &pf, Eigen::Vector3d &v0, Eigen::Vector3d &vf, double T) {
    auto T2 = T*T;
    auto T3 = T2*T;
    auto dp = pf - v0 * T - p0;
    auto dv = vf - v0;
    auto alpha = -12.0 / T3 * dp + 6.0 / T2 * dv;
    auto beta =  6.0 / T2 *  dp - 2.0 / T * dv;
    double J = T;
    for(int i = 0; i<3; i++) {
        J += alpha(i)*alpha(i)*T3 + alpha(i)*beta(i)*T2 + beta(i)*beta(i)*T;
    }
    return J;
}

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

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,Eigen::Vector3d _start_velocity,Eigen::Vector3d _target_position)
{
    double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it 
    /*
                    



    STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
    the solving process has been given in the document

    because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP

    after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory


    */
    Eigen::Vector3d target_velocity{0,0,0};

    double x = 10;

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
    
    optimal_cost = getCost(_start_position, _start_velocity, _target_position, target_velocity, x);
    // std::cout << summary.BriefReport() << "\n";
    // std::cout << "x :10 -> " << x << ", min cost: "<< optimal_cost <<"\n";

    // some codes for validation
    // double a, cost;
    // double min_a = 0, min_cost = 10000;
    // for(int i = 0; i<1000; i++) {
    //     auto a = i * 0.1;
    //     auto cost = getCost(_start_position, _start_velocity, _target_position, target_velocity, a);
    //     if (cost < min_cost) {
    //         min_a = a;
    //         min_cost = cost;
    //         //std::cout << "optimal x" << x << " (cost:" << optimal_cost << "), find another : "<<  i * 0.1 << " (cost:" << tmp << ")\n";
    //     }
    // }
    // std::cout << "optimal x" << x << " (cost:" << optimal_cost << "),  loop soulution : "<<  min_a << " (cost:" << min_cost << ")" << std::endl;
    return optimal_cost;
}
