#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>

/*
  SUB  : /global_planner/global_path   (nav_msgs::Path)
         /turtlebot3_waffle_pi/odom    (nav_msgs::Odometry)
  PUB  : /turtlebot3_waffle_pi/cmd_vel (geometry_msgs::Twist)
 */

class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController() : Node("controller", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    lookahead_min_  = declare_parameter("lookahead_min",  0.30);
    lookahead_max_  = declare_parameter("lookahead_max",  1.00);
    lookahead_gain_ = declare_parameter("lookahead_gain", 1.20);
    max_speed_      = declare_parameter("max_speed",      0.15);
    max_yaw_rate_   = declare_parameter("max_yaw_rate",   0.60);
    goal_tol_pos_   = declare_parameter("goal_tolerance", 0.05);
    goal_tol_yaw_   = declare_parameter("heading_tol",    0.20);

    rclcpp::QoS path_qos{rclcpp::KeepLast(10)};
    path_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    path_qos.durability (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/global_planner/global_path", path_qos,
        std::bind(&PurePursuitController::onPath, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/turtlebot3_waffle_pi/odom", rclcpp::QoS(50),
        std::bind(&PurePursuitController::onOdom, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtlebot3_waffle_pi/cmd_vel", 10);   //set as 10 can be increased

    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuitController::controlLoop, this));
  }

private:
  /* ----- callbacks ---------------------------------------------------- */
  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg->poses.empty()) { path_ = *msg; has_path_ = true; }
  }
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = *msg; has_odom_ = true;
  }

  /* ----- main loop ---------------------------------------------------- */
  void controlLoop()
  {
    if (!has_path_ || !has_odom_) return;

    /* current pose */
    const auto &po = odom_.pose.pose;
    const double rx   = po.position.x;
    const double ry   = po.position.y;
    const double ryaw = tf2::getYaw(po.orientation);

    /* goal data */
    const auto  goal_pose = path_.poses.back();
    const auto &goal_pt   = goal_pose.pose.position;
    const double dist_goal = std::hypot(goal_pt.x - rx, goal_pt.y - ry);

    /* goal handling */

    if (dist_goal < goal_tol_pos_)
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    /* dynamic look‑ahead */
    const double v = std::hypot(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y);
    double Ld = std::clamp(lookahead_gain_ * v, lookahead_min_, lookahead_max_);

    /* target selection (front points only) */
    geometry_msgs::msg::Point target = goal_pt;
    double best_dist = std::numeric_limits<double>::max();
    bool   found = false;

    const double bdx = std::cos(ryaw), bdy = std::sin(ryaw);

    for (const auto &ps : path_.poses)
    {
      double dx = ps.pose.position.x - rx;
      double dy = ps.pose.position.y - ry;
      if (dx*bdx + dy*bdy <= 0.0) continue;          // behind → skip
      double dist = std::hypot(dx, dy);
      if (dist >= Ld && dist < best_dist)
      {
        best_dist = dist; target = ps.pose.position; found = true;
      }
    }
    if (!found && dist_goal < Ld)
    {
      target = goal_pt; Ld = std::max(dist_goal, 0.01);
    }

    /* pursuit steering */
    const double dx = target.x - rx, dy = target.y - ry;
    double alpha = normalize(std::atan2(dy, dx) - ryaw);
    double steer = std::clamp(2.0 * std::sin(alpha) / Ld, -max_yaw_rate_, max_yaw_rate_);

    /* linear speed */
    double linear = max_speed_ * (1.0 - std::min(std::abs(alpha)/(M_PI/2.0), 0.9));
    if (std::abs(alpha) > M_PI/2.0) linear = 0; // 헤딩 오차 90° 초과 시 전진 정지 → 제자리 회전만

    linear = std::max(linear, 0.0);  //후진 x

    geometry_msgs::msg::Twist cmd; cmd.linear.x = linear; cmd.angular.z = steer;
    cmd_pub_->publish(cmd);
  }

  /* util */
  static double normalize(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  /* members */
  nav_msgs::msg::Path      path_;
  nav_msgs::msg::Odometry  odom_;
  bool has_path_{false}, has_odom_{false};

  double lookahead_min_{}, lookahead_max_{}, lookahead_gain_{};
  double max_speed_{}, max_yaw_rate_{};
  double goal_tol_pos_{}, goal_tol_yaw_{};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitController>());
  rclcpp::shutdown();
  return 0;
} 
