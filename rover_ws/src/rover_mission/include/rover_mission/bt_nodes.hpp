// Custom BehaviorTree.CPP (v3) nodes for the rover mission logic.
//
// These are the rover's OWN mission code -- not an installed package. The tree
// (bt/mission.xml) is ticked by mission_server, which wires the shared runtime
// state (ROS node, waypoint queue, threat/mode flags) onto the blackboard.
#ifndef ROVER_MISSION__BT_NODES_HPP_
#define ROVER_MISSION__BT_NODES_HPP_

#include <array>
#include <deque>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

namespace rover_mission
{

using Waypoint = std::array<double, 3>;          // x, y, yaw
using WaypointQueue = std::deque<Waypoint>;
using WaypointQueuePtr = std::shared_ptr<WaypointQueue>;

// Fetch the shared rclcpp node stored on the blackboard by mission_server.
rclcpp::Node::SharedPtr blackboardNode(const BT::TreeNode & self);

// ---------------------------------------------------------------------------
// Navigation actions (rclcpp_action clients to Nav2's /navigate_to_pose)
// ---------------------------------------------------------------------------
class NavigateBase : public BT::StatefulActionNode
{
public:
  NavigateBase(const std::string & name, const BT::NodeConfiguration & config);

protected:
  using ActionT = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  // Subclasses provide the goal in the global frame.
  virtual bool getGoal(double & x, double & y, double & yaw) = 0;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  bool result_received_{false};
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
  std::string frame_{"odom"};
  // If the action server's name is graph-discoverable but its Nav2 BT
  // executor isn't actually running yet (possible right after Nav2's
  // lifecycle bring-up, since a LifecycleNode's action server object exists
  // once configured, before it's activated), a goal sent to it can be
  // accepted-but-never-processed: goal_response_callback simply never fires.
  // Track when we sent the goal so onRunning() can fail fast instead of
  // waiting forever for a response that will never come.
  rclcpp::Time goal_sent_time_;
  // Set true the instant goal_response_callback fires, whether or not the
  // goal was actually accepted -- lets onRunning() tell "rejected" (fail
  // fast and retry) apart from "no response yet" (genuinely still waiting),
  // which a bare null check on goal_handle_ can't distinguish.
  bool response_received_{false};
  static constexpr double kGoalAcceptTimeoutSec = 15.0;
};

// Drive to the current waypoint (ports x, y, yaw provided by GetNextWaypoint).
class NavigateToPose : public NavigateBase
{
public:
  NavigateToPose(const std::string & name, const BT::NodeConfiguration & config)
  : NavigateBase(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("x"), BT::InputPort<double>("y"),
            BT::InputPort<double>("yaw")};
  }

protected:
  bool getGoal(double & x, double & y, double & yaw) override;
};

// Drive to the home pose (read from the blackboard).
class ReturnToHome : public NavigateBase
{
public:
  ReturnToHome(const std::string & name, const BT::NodeConfiguration & config)
  : NavigateBase(name, config) {}
  static BT::PortsList providedPorts() {return {};}

protected:
  bool getGoal(double & x, double & y, double & yaw) override;
};

// ---------------------------------------------------------------------------
// Waypoint queue management
// ---------------------------------------------------------------------------
// Returns the active waypoint. Pops a new one only when the previous goal has
// been cleared (via ClearGoalFlag), so a reactive parent can re-tick safely.
class GetNextWaypoint : public BT::SyncActionNode
{
public:
  GetNextWaypoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<double>("x"), BT::OutputPort<double>("y"),
            BT::OutputPort<double>("yaw")};
  }
  BT::NodeStatus tick() override;
};

// Marks the current goal as consumed so the next tick pops a fresh waypoint.
class ClearGoalFlag : public BT::SyncActionNode
{
public:
  ClearGoalFlag(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// Stop-scan-mark-reroute + drone actions
// ---------------------------------------------------------------------------
class StopMotion : public BT::SyncActionNode
{
public:
  StopMotion(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// Rasterise the current detected threat into a keepout exclusion polygon,
// published to /threat/add_zone (Nav2's KeepoutFilter then forces a replan).
class MarkThreat : public BT::SyncActionNode
{
public:
  MarkThreat(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;
};

class ClearThreatFlag : public BT::SyncActionNode
{
public:
  ClearThreatFlag(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;
};

class LaunchDrone : public BT::SyncActionNode
{
public:
  LaunchDrone(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_;
  bool launched_{false};
};

// ---------------------------------------------------------------------------
// Conditions (read shared flags off the blackboard)
// ---------------------------------------------------------------------------
class IsThreatDetected : public BT::ConditionNode
{
public:
  IsThreatDetected(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;
};

class IsTerrainBlind : public BT::ConditionNode
{
public:
  IsTerrainBlind(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;
};

class IsMode : public BT::ConditionNode
{
public:
  IsMode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mode")};
  }
  BT::NodeStatus tick() override;
};

// Register every custom node with the factory.
void registerNodes(BT::BehaviorTreeFactory & factory);

}  // namespace rover_mission

#endif  // ROVER_MISSION__BT_NODES_HPP_
