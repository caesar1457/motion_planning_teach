#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "car_msgs/msg/car_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "mpc_car/mpc_car.hpp"

namespace mpc_car {
class MpcCarNode
{
public:
  explicit MpcCarNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) {
    // create node
    node_ = std::make_shared<rclcpp::Node>("mpc_car", options);
    // params
    double dt = 0;
  
    node_->declare_parameter("dt", dt);
    node_->declare_parameter("delay", delay_);
    node_->get_parameter("dt", dt);
    node_->get_parameter("delay", delay_);
    // 
    mpcPtr_ = std::make_shared<MpcCar>(node_);
    cmd_pub_ = node_->create_publisher<car_msgs::msg::CarCmd>("car_cmd", 1);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, std::bind(&MpcCarNode::odom_call_back, this, std::placeholders::_1));
    using namespace std::literals;
    plan_timer_ = node_->create_wall_timer(1s * dt, std::bind(&MpcCarNode::plan_timer_callback, this));
  }
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }
  void plan_timer_callback() {
    if (init_) {
      rclcpp::Time t1 = node_->get_clock()->now();
      auto ret = mpcPtr_->solveQP(state_);
      // assert(ret == 1);
      rclcpp::Time t2 = node_->get_clock()->now();
      double solve_time = (t2 - t1).seconds();
      std::cout << "solve qp costs: " << 1e3 * solve_time << "ms" << std::endl;
      // TODO
      car_msgs::msg::CarCmd msg;
      msg.header.frame_id = "world";
      msg.header.stamp = node_->get_clock()->now();
      if (ret == 1) {
        VectorX x;
        VectorU u;
        mpcPtr_->getPredictXU(0, x, u);
        std::cout << "u: " << u.transpose() << std::endl;
        std::cout << "x: " << x.transpose() << std::endl;
        std::cout << std::endl;
        msg.a = u(0);
        msg.delta = u(1);
      } else {
        msg.a = 0;
        msg.delta = 0;
      }

      cmd_pub_->publish(msg);
      mpcPtr_->visualization();
    }
    return;
  }
  void odom_call_back(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    state_ << x, y, euler.z(), v.norm();
    init_ = true;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MpcCar> mpcPtr_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::Publisher<car_msgs::msg::CarCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  VectorX state_;
  bool init_ = false;
  double delay_ = 0.0;
};
}  // namespace mpc_car

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mpc_car::MpcCarNode)
