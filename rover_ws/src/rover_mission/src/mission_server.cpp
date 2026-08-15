// mission_server: loads the BehaviorTree.CPP mission tree, wires shared runtime
// state onto the blackboard, and ticks the tree while pumping ROS callbacks.
//
// Operational modes (set via /mission/mode, std_msgs/String):
//   Standard              - supervised operation (mission runs; operator in loop).
//   Autonomous            - full autonomy following waypoints + RoE.
//   Ghost                 - zero RF: suppress the telemetry publisher.
//   EmergencyReturn       - abort mission and return to home (BT override branch).
//   LostCommsAutonomous   - continue the mission autonomously without a link.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "rover_mission/bt_nodes.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mission_server");

  // --- Parameters ---
  node->declare_parameter<std::string>("bt_xml", "");
  node->declare_parameter<std::string>("global_frame", "odom");
  node->declare_parameter<std::string>("initial_mode", "Autonomous");
  node->declare_parameter<double>("threat_radius", 1.0);
  node->declare_parameter<double>("tick_rate_hz", 10.0);
  // Waypoints flattened as [x1, y1, yaw1, x2, y2, yaw2, ...].
  node->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("home", std::vector<double>{0.0, 0.0, 0.0});

  const std::string bt_xml = node->get_parameter("bt_xml").as_string();
  const std::string global_frame = node->get_parameter("global_frame").as_string();
  const std::string initial_mode = node->get_parameter("initial_mode").as_string();
  const double threat_radius = node->get_parameter("threat_radius").as_double();
  const double tick_rate = node->get_parameter("tick_rate_hz").as_double();
  const auto wp_flat = node->get_parameter("waypoints").as_double_array();
  const auto home = node->get_parameter("home").as_double_array();

  if (bt_xml.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter 'bt_xml' is required.");
    return 1;
  }

  // --- Build the shared waypoint queue ---
  auto queue = std::make_shared<rover_mission::WaypointQueue>();
  for (size_t i = 0; i + 3 <= wp_flat.size(); i += 3) {
    queue->push_back({wp_flat[i], wp_flat[i + 1], wp_flat[i + 2]});
  }
  RCLCPP_INFO(node->get_logger(), "Loaded %zu waypoints.", queue->size());

  // --- Blackboard shared state ---
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("waypoints", queue);
  blackboard->set("global_frame", global_frame);
  blackboard->set("have_goal", false);
  blackboard->set("cur_x", 0.0);
  blackboard->set("cur_y", 0.0);
  blackboard->set("cur_yaw", 0.0);
  blackboard->set("threat_active", false);
  blackboard->set("threat_x", 0.0);
  blackboard->set("threat_y", 0.0);
  blackboard->set("threat_radius", threat_radius);
  blackboard->set("terrain_blind", false);
  blackboard->set("mode", initial_mode);
  blackboard->set("home_x", home.size() > 0 ? home[0] : 0.0);
  blackboard->set("home_y", home.size() > 1 ? home[1] : 0.0);
  blackboard->set("home_yaw", home.size() > 2 ? home[2] : 0.0);

  // --- Build the tree ---
  BT::BehaviorTreeFactory factory;
  rover_mission::registerNodes(factory);
  BT::Tree tree = factory.createTreeFromFile(bt_xml, blackboard);

  // --- Subscriptions that feed the blackboard ---
  auto threat_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/detection/threat", 10,
    [blackboard, node](geometry_msgs::msg::PointStamped::SharedPtr msg) {
      blackboard->set("threat_active", true);
      blackboard->set("threat_x", msg->point.x);
      blackboard->set("threat_y", msg->point.y);
      RCLCPP_INFO(node->get_logger(), "Threat reported at (%.2f, %.2f).",
        msg->point.x, msg->point.y);
    });

  auto loc_mode_sub = node->create_subscription<std_msgs::msg::String>(
    "/localization/mode", 10,
    [blackboard](std_msgs::msg::String::SharedPtr msg) {
      blackboard->set("terrain_blind", msg->data == "DEGRADED_DEAD_RECKONING");
    });

  auto mode_sub = node->create_subscription<std_msgs::msg::String>(
    "/mission/mode", 10,
    [blackboard, node](std_msgs::msg::String::SharedPtr msg) {
      blackboard->set("mode", msg->data);
      RCLCPP_INFO(node->get_logger(), "Operational mode -> %s", msg->data.c_str());
    });

  auto status_pub = node->create_publisher<std_msgs::msg::String>("/mission/status", 10);

  // --- Tick loop (single-threaded: ticks and callbacks are serialised) ---
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  rclcpp::Rate rate(tick_rate);

  // Nav2's action server names (navigate_to_pose etc.) become graph-discoverable
  // as soon as their owning lifecycle nodes are merely *configured*, well before
  // they're *activated* -- wait_for_action_server() alone can't tell the
  // difference, and a goal sent to a configured-but-not-yet-active server gets
  // rejected outright rather than queued. In practice Nav2's full lifecycle
  // bring-up (all 6 servers) has consistently taken ~20-25s in testing, more
  // under CPU load. Give it a comfortable margin here rather than let the
  // mission tree hammer real navigation attempts against a server that isn't
  // ready yet -- observed to cascade into repeated goal rejections and, once
  // finally accepted, the previous accepted goal's just-aborted state leaving
  // planner_server briefly unable to route at all.
  constexpr double kNav2StartupGraceSec = 30.0;
  RCLCPP_INFO(node->get_logger(),
    "Mission server up; waiting %.0fs for Nav2 to finish activating before "
    "the mission tree starts driving...", kNav2StartupGraceSec);
  {
    const auto deadline = node->now() + rclcpp::Duration::from_seconds(kNav2StartupGraceSec);
    rclcpp::Rate grace_rate(tick_rate);
    while (rclcpp::ok() && node->now() < deadline) {
      exec.spin_some();
      grace_rate.sleep();
    }
  }
  RCLCPP_INFO(node->get_logger(), "Mission server running (mode=%s).",
    initial_mode.c_str());

  while (rclcpp::ok()) {
    tree.tickRoot();
    exec.spin_some();

    // Telemetry is suppressed in Ghost mode (zero RF emission).
    std::string mode;
    blackboard->get("mode", mode);
    if (mode != "Ghost") {
      std_msgs::msg::String status;
      status.data = "mode=" + mode;
      status_pub->publish(status);
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
