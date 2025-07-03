#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>

/*
  SUB  : /global_planner/global_path   (nav_msgs::Path)
         /turtlebot3_waffle_pi/odom    (nav_msgs::Odometry)
         /goal_pose                    (geometry_msgs::PoseStamped) 
  PUB  : /turtlebot3_waffle_pi/cmd_vel (geometry_msgs::Twist)
*/

class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController()
      : Node("controller",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    lookahead_min_  = declare_parameter("lookahead_min",  0.30);
    lookahead_max_  = declare_parameter("lookahead_max",  1.00);
    lookahead_gain_ = declare_parameter("lookahead_gain", 1.20);
    max_speed_      = declare_parameter("max_speed",      0.15);
    max_yaw_rate_   = declare_parameter("max_yaw_rate",   0.60);
    goal_tol_pos_   = declare_parameter("goal_tolerance", 0.05);
    goal_tol_yaw_   = declare_parameter("heading_tol",    0.20);

    // Sub, Pub
    rclcpp::QoS path_qos{rclcpp::KeepLast(10)};
    path_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    path_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/global_planner/global_path", path_qos,
        std::bind(&PurePursuitController::onPath, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/turtlebot3_waffle_pi/odom", rclcpp::QoS(50),
        std::bind(&PurePursuitController::onOdom, this, std::placeholders::_1));

    // goal pose는 선택 사항 → 있으면 사용, 없으면 path 끝점을 사용
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&PurePursuitController::onGoalPose, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/turtlebot3_waffle_pi/cmd_vel", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PurePursuitController::controlLoop, this));
  }

private:
//callback
  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg->poses.empty()) {
      path_ = *msg;
      has_path_ = true;
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = *msg;
    has_odom_ = true;
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
  }

  void controlLoop()
  {
    if (!has_path_ || !has_odom_) return; 

    // 현재 pose
    const auto &po   = odom_.pose.pose;
    const double rx  = po.position.x;
    const double ry  = po.position.y;
    const double ryaw = tf2::getYaw(po.orientation);

      // goal_pose 후 경로 마지막 지점 - 목표지점 설정
      geometry_msgs::msg::Point goal_pt;
      if (has_goal_)
        goal_pt = goal_pose_.pose.position;          // /goal_pose 우선
      else
        goal_pt = path_.poses.back().pose.position;  // 경로 마지막 점으로 설정

    const double dist_goal = std::hypot(goal_pt.x - rx, goal_pt.y - ry);

    // goal도달 -> 이동x
    if (dist_goal < goal_tol_pos_)
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    // Look ahead distance
    const double v  = std::hypot(odom_.twist.twist.linear.x,
                                 odom_.twist.twist.linear.y);
    double Ld = std::clamp(lookahead_gain_ * v, lookahead_min_, lookahead_max_);

    // track point
    geometry_msgs::msg::Point target = goal_pt;
    double best_dist = std::numeric_limits<double>::max();
    const double bdx = std::cos(ryaw), bdy = std::sin(ryaw);

    for (const auto &ps : path_.poses)
    {
      const double dx = ps.pose.position.x - rx;
      const double dy = ps.pose.position.y - ry;
      if (dx*bdx + dy*bdy <= 0.0) continue;        // 뒤쪽 -> 무시
      const double dist = std::hypot(dx, dy);
      if (dist >= Ld && dist < best_dist)
      {
        best_dist = dist;
        target    = ps.pose.position;
      }
    }
    if (best_dist == std::numeric_limits<double>::max() && dist_goal < Ld)
    {
      target = goal_pt;
      Ld     = std::max(dist_goal, 0.01);
    }

    // 스티어링
    const double dx    = target.x - rx;
    const double dy    = target.y - ry;
    double alpha       = normalize(std::atan2(dy, dx) - ryaw);
    double steer       = std::clamp(2.0 * std::sin(alpha) / Ld,
                                    -max_yaw_rate_, max_yaw_rate_);

    // 속도
    double linear = max_speed_ * (1.0 - std::min(std::abs(alpha)/(M_PI/2.0), 0.9));
    if (std::abs(alpha) > M_PI/2.0) linear = 0;  // 90° 초과 → 회전 우선
    linear = std::max(linear, 0.0);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear;
    cmd.angular.z = steer;
    cmd_pub_->publish(cmd);
  }

  //각도 
  static double normalize(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  nav_msgs::msg::Path      path_;
  nav_msgs::msg::Odometry  odom_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  bool has_path_{false};
  bool has_odom_{false};
  bool has_goal_{false};

  double lookahead_min_{};
  double lookahead_max_{};
  double lookahead_gain_{};
  double max_speed_{};
  double max_yaw_rate_{};
  double goal_tol_pos_{};
  double goal_tol_yaw_{};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
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
