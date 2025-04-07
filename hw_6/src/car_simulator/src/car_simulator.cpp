#include <deque>

#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include "car_msgs/msg/car_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"

struct Car {
  double l;
  Eigen::Vector4d state;
  // state: x, y, phi, v
  // input: a, delta
  inline void setInitialState(const Eigen::Vector4d& s) {
    state = s;
  }
  inline Eigen::Vector4d diff(const Eigen::Vector4d& s,
                              const Eigen::Vector2d& input) const {
    Eigen::Vector4d ds;
    double phi = s(2);
    double v = s(3);
    double a = input(0);
    double delta = input(1);
    ds(0) = v * cos(phi);
    ds(1) = v * sin(phi);
    ds(2) = v / l * tan(delta);
    ds(3) = a;
    return ds;
  }

  void step(const Eigen::Vector2d& input, const double dt) {
    // Runge–Kutta
    Eigen::Vector4d k1 = diff(state, input);
    Eigen::Vector4d k2 = diff(state + k1 * dt / 2, input);
    Eigen::Vector4d k3 = diff(state + k1 * dt / 2, input);
    Eigen::Vector4d k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
  }
};

namespace car_simulator {
class CarSimulator : public rclcpp::Node {
 public:
  CarSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("car_simulator", options)
   {
    // param
    Eigen::Vector4d initS;
    declare_parameter("l", car.l);
    declare_parameter("x", initS(0));
    declare_parameter("y", initS(1));
    declare_parameter("phi", initS(2));
    declare_parameter("v", initS(3));
    declare_parameter("delay", delay_);
    get_parameter("l", car.l);
    get_parameter("x", initS(0));
    get_parameter("y", initS(1));
    get_parameter("phi", initS(2));
    get_parameter("v", initS(3));
    get_parameter("delay", delay_);

    // init
    input_.setZero();
    car.setInitialState(initS);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    cmd_sub_ = create_subscription<car_msgs::msg::CarCmd>(
      "car_cmd", 1, std::bind(&CarSimulator::cmd_callback, this, std::placeholders::_1));
    using namespace std::literals;
    sim_timer_ = create_wall_timer(1s * (1.0 / 400), std::bind(&CarSimulator::timer_callback, this));
  }

  struct DelayedMsg {
    rclcpp::Time t;
    double a, delta;
    DelayedMsg() {}
    DelayedMsg(const rclcpp::Time& _t, double _a, double _delta) : t(_t), a(_a), delta(_delta) {}
  };
  std::deque<DelayedMsg> delayedMsgs_;

  void cmd_callback(const car_msgs::msg::CarCmd::SharedPtr msg) {
    delayedMsgs_.emplace_back(get_clock()->now(), msg->a, msg->delta);
    // input_(0) = msg->a;
    // input_(1) = msg->delta;
  }
  void timer_callback() {
    if (!delayedMsgs_.empty()) {
      auto& msg = delayedMsgs_.front();
      if ((get_clock()->now() - msg.t).seconds() > delay_) {
        input_(0) = msg.a;
        input_(1) = msg.delta;
        delayedMsgs_.pop_front();
      }
    }

    car.step(input_, 1.0 / 400);
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = "world";
    odom_msg.pose.pose.position.x = car.state(0);
    odom_msg.pose.pose.position.y = car.state(1);
    odom_msg.pose.pose.position.z = 0.0;
    double phi = car.state(2);
    double v = car.state(3);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(phi / 2);
    odom_msg.pose.pose.orientation.w = cos(phi / 2);

    odom_msg.twist.twist.linear.x = v * cos(phi);
    odom_msg.twist.twist.linear.y = v * sin(phi);
    odom_msg.twist.twist.linear.z = 0.0;

    odom_pub_->publish(odom_msg);
  }
 private:
  Car car;
  double delay_ = 0.0;
  Eigen::Vector2d input_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<car_msgs::msg::CarCmd>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
};
}  // namespace car_simulator

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(car_simulator::CarSimulator)
