#include "rover_mission/bt_nodes.hpp"

#include <cmath>

namespace rover_mission
{

rclcpp::Node::SharedPtr blackboardNode(const BT::TreeNode & self)
{
  rclcpp::Node::SharedPtr node;
  self.config().blackboard->get("node", node);
  return node;
}

static void setQuaternionYaw(geometry_msgs::msg::Quaternion & q, double yaw)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
}

// =========================================================================
// NavigateBase
// =========================================================================
NavigateBase::NavigateBase(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = blackboardNode(*this);
  config.blackboard->get("global_frame", frame_);
  client_ = rclcpp_action::create_client<ActionT>(node_, "navigate_to_pose");
}

BT::NodeStatus NavigateBase::onStart()
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!getGoal(x, y, yaw)) {
    return BT::NodeStatus::FAILURE;
  }
  // 15s, not 2s: mission_server starts ticking its tree immediately, but
  // Nav2's bt_navigator can easily take longer than 2s to finish its
  // lifecycle bring-up (more under CPU load, e.g. with the Gazebo/RViz GUI
  // also running). A too-short wait here doesn't retry -- it fails the whole
  // follow_waypoint sequence outright, which (a) skips ClearGoalFlag, so
  // have_goal is stuck true and the real waypoint gets abandoned, and
  // (b) falls through to ReturnToHome, whose target is ~the spawn pose, so
  // Nav2 ends up planning a ~zero-length path and never actually finishes --
  // the rover looks like it's simply not driving at all. Only the first call
  // per action client actually needs to wait; once matched, later calls
  // return immediately, so the higher ceiling costs nothing in steady state.
  if (!client_->wait_for_action_server(std::chrono::seconds(15))) {
    RCLCPP_WARN(node_->get_logger(), "navigate_to_pose action server unavailable");
    return BT::NodeStatus::FAILURE;
  }

  result_received_ = false;
  result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  goal_handle_.reset();

  ActionT::Goal goal;
  goal.pose.header.frame_id = frame_;
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  setQuaternionYaw(goal.pose.pose.orientation, yaw);

  auto options = rclcpp_action::Client<ActionT>::SendGoalOptions();
  options.goal_response_callback =
    [this](GoalHandle::SharedPtr handle) {goal_handle_ = handle;};
  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      result_received_ = true;
      result_code_ = result.code;
    };
  client_->async_send_goal(goal, options);

  RCLCPP_INFO(node_->get_logger(), "Navigating to (%.2f, %.2f, %.2f rad)", x, y, yaw);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateBase::onRunning()
{
  if (result_received_) {
    return (result_code_ == rclcpp_action::ResultCode::SUCCEEDED)
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateBase::onHalted()
{
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
}

bool NavigateToPose::getGoal(double & x, double & y, double & yaw)
{
  auto rx = getInput<double>("x");
  auto ry = getInput<double>("y");
  auto ryaw = getInput<double>("yaw");
  if (!rx || !ry) {
    return false;
  }
  x = rx.value();
  y = ry.value();
  yaw = ryaw ? ryaw.value() : 0.0;
  return true;
}

bool ReturnToHome::getGoal(double & x, double & y, double & yaw)
{
  auto bb = config().blackboard;
  bb->get("home_x", x);
  bb->get("home_y", y);
  bb->get("home_yaw", yaw);
  RCLCPP_INFO(node_->get_logger(), "Return-to-home engaged.");
  return true;
}

// =========================================================================
// Waypoint queue
// =========================================================================
BT::NodeStatus GetNextWaypoint::tick()
{
  auto bb = config().blackboard;
  bool have_goal = false;
  bb->get("have_goal", have_goal);

  if (!have_goal) {
    WaypointQueuePtr queue;
    bb->get("waypoints", queue);
    if (!queue || queue->empty()) {
      return BT::NodeStatus::FAILURE;   // no more waypoints -> parent falls back
    }
    const Waypoint wp = queue->front();
    queue->pop_front();
    bb->set("cur_x", wp[0]);
    bb->set("cur_y", wp[1]);
    bb->set("cur_yaw", wp[2]);
    bb->set("have_goal", true);
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  bb->get("cur_x", x);
  bb->get("cur_y", y);
  bb->get("cur_yaw", yaw);
  setOutput("x", x);
  setOutput("y", y);
  setOutput("yaw", yaw);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ClearGoalFlag::tick()
{
  config().blackboard->set("have_goal", false);
  return BT::NodeStatus::SUCCESS;
}

// =========================================================================
// Stop / mark / drone
// =========================================================================
StopMotion::StopMotion(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = blackboardNode(*this);
  pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::NodeStatus StopMotion::tick()
{
  pub_->publish(geometry_msgs::msg::Twist());
  RCLCPP_INFO(node_->get_logger(), "STOP: threat encountered, halting to scan.");
  return BT::NodeStatus::SUCCESS;
}

MarkThreat::MarkThreat(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = blackboardNode(*this);
  pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>("/threat/add_zone", 10);
}

BT::NodeStatus MarkThreat::tick()
{
  auto bb = config().blackboard;
  double tx = 0.0, ty = 0.0, r = 1.0;
  std::string frame = "odom";
  bb->get("threat_x", tx);
  bb->get("threat_y", ty);
  bb->get("threat_radius", r);
  bb->get("global_frame", frame);

  geometry_msgs::msg::PolygonStamped poly;
  poly.header.frame_id = frame;
  poly.header.stamp = node_->now();
  const double dx[4] = {-r, r, r, -r};
  const double dy[4] = {-r, -r, r, r};
  for (int i = 0; i < 4; ++i) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(tx + dx[i]);
    p.y = static_cast<float>(ty + dy[i]);
    p.z = 0.0f;
    poly.polygon.points.push_back(p);
  }
  pub_->publish(poly);
  RCLCPP_WARN(node_->get_logger(),
    "MARK: threat at (%.2f, %.2f) -> keepout zone, forcing reroute.", tx, ty);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ClearThreatFlag::tick()
{
  config().blackboard->set("threat_active", false);
  return BT::NodeStatus::SUCCESS;
}

LaunchDrone::LaunchDrone(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = blackboardNode(*this);
  pub_ = node_->create_publisher<std_msgs::msg::Empty>("/drone/launch", 10);
}

BT::NodeStatus LaunchDrone::tick()
{
  // Edge-triggered: only fire once per blind episode.
  if (!launched_) {
    pub_->publish(std_msgs::msg::Empty());
    RCLCPP_WARN(node_->get_logger(), "LAUNCH DRONE: terrain blind, deploying scout.");
    launched_ = true;
  }
  return BT::NodeStatus::SUCCESS;
}

// =========================================================================
// Conditions
// =========================================================================
BT::NodeStatus IsThreatDetected::tick()
{
  bool active = false;
  config().blackboard->get("threat_active", active);
  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsTerrainBlind::tick()
{
  bool blind = false;
  config().blackboard->get("terrain_blind", blind);
  return blind ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsMode::tick()
{
  std::string want;
  if (!getInput<std::string>("mode", want)) {
    return BT::NodeStatus::FAILURE;
  }
  std::string current;
  config().blackboard->get("mode", current);
  return (want == current) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// =========================================================================
void registerNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<ReturnToHome>("ReturnToHome");
  factory.registerNodeType<GetNextWaypoint>("GetNextWaypoint");
  factory.registerNodeType<ClearGoalFlag>("ClearGoalFlag");
  factory.registerNodeType<StopMotion>("StopMotion");
  factory.registerNodeType<MarkThreat>("MarkThreat");
  factory.registerNodeType<ClearThreatFlag>("ClearThreatFlag");
  factory.registerNodeType<LaunchDrone>("LaunchDrone");
  factory.registerNodeType<IsThreatDetected>("IsThreatDetected");
  factory.registerNodeType<IsTerrainBlind>("IsTerrainBlind");
  factory.registerNodeType<IsMode>("IsMode");
}

}  // namespace rover_mission
