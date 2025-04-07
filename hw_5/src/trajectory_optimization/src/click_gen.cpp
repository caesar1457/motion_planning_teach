#include "trajectory_optimization/visualizer.hpp"
#include "trajectory_optimization/trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Sparse>	
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>

struct Config
{
    std::string targetTopic;
    double clickHeight;
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;
    int maxPieceNum;
};

double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return 2.0 * sqrt(dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

int getAnm(int n, int m) {
    int a = 1;
    while(m > 0) {
        a = a * n;
        n--;
        m--;
    }
    return a;
}

Eigen::VectorXd getPloyVector(double t, int order){
    Eigen::VectorXd v = Eigen::VectorXd::Zero(6);
    for(int i = order; i < 6; i++) {
        v(i) = getAnm(i, order) * std::pow(t, i - order);
    }
    return v;
}

Eigen::MatrixXd getE(double t){
    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(6, 6);
    for(int i = 1; i < 6; i++) {
        E.row(i) = getPloyVector(t, i-1);
    }
    return E;
}

Eigen::MatrixXd getF(){
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6);
    F(0, 0) = 1;
    for (int i = 1; i < 6; i++) {
        F.row(i) = -getPloyVector(0, i-1);
    }
    return F;
}


void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix)
{   // TODO : Implement this function
    // coefficientMatrix is a matrix with 6*piece num rows and 3 columes
    // As for a polynomial c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5,
    // each 6*3 sub-block of coefficientMatrix is
    // --              --
    // | c0_x c0_y c0_z |
    // | c1_x c1_y c1_z |
    // | c2_x c2_y c2_z |
    // | c3_x c3_y c3_z |
    // | c4_x c4_y c4_z |
    // | c5_x c5_y c5_z |
    // --              --
    // Please computed coefficientMatrix of the minimum-jerk trajectory
    // in this function

    // ------------------------ Put your solution below ------------------------

    
    // ------------------------ Put your solution above ------------------------
}

class ClickGen
{
private:
    Config config_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

    Visualizer visualizer_;

    Eigen::Matrix3Xd positions_;
    Eigen::VectorXd times_;
    int position_num_{0};
    Trajectory<5> traj_;

public:
    ClickGen(rclcpp::Node::SharedPtr node)
        : node_(node),
          visualizer_(node)
    {
        // params
        node_->declare_parameter("TargetTopic", config_.targetTopic);
        node_->declare_parameter("ClickHeight", config_.clickHeight);
        node_->declare_parameter("InitialVel", config_.initialVel);
        node_->declare_parameter("InitialAcc", config_.initialAcc);
        node_->declare_parameter("TerminalVel", config_.terminalVel);
        node_->declare_parameter("TerminalAcc", config_.terminalAcc);
        node_->declare_parameter("AllocationSpeed", config_.allocationSpeed);
        node_->declare_parameter("AllocationAcc", config_.allocationAcc);
        node_->declare_parameter("MaxPieceNum", config_.maxPieceNum);
        node_->get_parameter("TargetTopic", config_.targetTopic);
        node_->get_parameter("ClickHeight", config_.clickHeight);
        node_->get_parameter("InitialVel", config_.initialVel);
        node_->get_parameter("InitialAcc", config_.initialAcc);
        node_->get_parameter("TerminalVel", config_.terminalVel);
        node_->get_parameter("TerminalAcc", config_.terminalAcc);
        node_->get_parameter("AllocationSpeed", config_.allocationSpeed);
        node_->get_parameter("AllocationAcc", config_.allocationAcc);
        node_->get_parameter("MaxPieceNum", config_.maxPieceNum);
        //
        positions_ = Eigen::Matrix3Xd(3, config_.maxPieceNum + 1);
        times_ = Eigen::VectorXd(config_.maxPieceNum);
        using namespace std::placeholders;
        target_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            config_.targetTopic, 1, std::bind(&ClickGen::targetCallBack, this, _1));
    }

    void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (position_num_ > config_.maxPieceNum)
        {
            position_num_ = 0;
            traj_.clear();
        }

        positions_(0, position_num_) = msg->pose.position.x;
        positions_(1, position_num_) = msg->pose.position.y;
        positions_(2, position_num_) = std::fabs(msg->pose.orientation.z) * config_.clickHeight;

        if (position_num_ > 0)
        {
            const double dist = (positions_.col(position_num_) - positions_.col(position_num_ - 1)).norm();
            times_(position_num_ - 1) = timeTrapzVel(dist, config_.allocationSpeed, config_.allocationAcc);
        }

        ++position_num_;

        if (position_num_ > 1)
        {
            const int pieceNum = position_num_ - 1;
            const Eigen::Vector3d initialPos = positions_.col(0);
            const Eigen::Vector3d initialVel(config_.initialVel[0], config_.initialVel[1], config_.initialVel[2]);
            const Eigen::Vector3d initialAcc(config_.initialAcc[0], config_.initialAcc[1], config_.initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions_.col(pieceNum);
            const Eigen::Vector3d terminalVel(config_.terminalVel[0], config_.terminalVel[1], config_.terminalVel[2]);
            const Eigen::Vector3d terminalAcc(config_.terminalAcc[0], config_.terminalAcc[1], config_.terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions_.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times_.head(pieceNum);

            Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

            minimumJerkTrajGen(pieceNum,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);

            traj_.clear();
            traj_.reserve(pieceNum);
            for (int i = 0; i < pieceNum; i++)
            {
                traj_.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
        }

        visualizer_.visualize(traj_, positions_.leftCols(position_num_));

        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("click_gen_node");
    auto click_gen_node = std::make_shared<ClickGen>(node);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}