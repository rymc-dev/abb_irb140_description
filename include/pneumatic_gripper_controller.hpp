#ifndef PNEUMATIC_GRIPPER_CONTROLLER_HPP
#define PNEUMATIC_GRIPPER_CONTROLLER_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

/**
 * ROS 2 node that drives the ABB IRB140 pneumatic gripper.
 *
 * It exposes a `SetBool` service (`~/gripper_trigger`) and, on each request,
 * toggles a digital output signal on the ABB controller through the Robot Web
 * Services (RWS) REST API using HTTP digest authentication.
 *
 *   data = true  -> lvalue 1 -> close gripper
 *   data = false -> lvalue 0 -> open gripper
 *
 * This is the C++ port of scripts/gripper_trigger_node.py. When the `sim`
 * parameter is true the HTTP call is skipped and every request succeeds, so the
 * same node can be used against the real controller or in simulation.
 *
 * On the real robot (`sim` false) nothing else publishes joint states for the
 * 12 pneumatic gripper finger joints (in sim, gz_ros2_control +
 * joint_state_broadcaster do). So when `sim` is false this node also mocks
 * those joint states on /joint_states: every finger joint sits at
 * `open_position` until a close command succeeds, then at `closed_position`
 * until an open command succeeds. robot_state_publisher merges this with the
 * arm joint states coming from the ABB bridge.
 */
class PneumaticGripperController : public rclcpp::Node
{
public:
  PneumaticGripperController();

private:
  // --- Fixed controller configuration ------------------------------------
  static constexpr const char * DEFAULT_ROBOT_IP = "192.168.125.1";
  static constexpr double DEFAULT_GRIPPER_EFFORT_PERCENTAGE = 50.0;
  static constexpr const char * USERNAME = "ros_cli";
  static constexpr const char * PASSWORD = "abb_irb140";
  static constexpr const char * SIGNAL_NAME = "closeGrippersOut";
  static constexpr double TIMEOUT = 5.0;

  // Mocked finger joint state (only published when `sim` is false).
  static constexpr double DEFAULT_OPEN_POSITION = 0.0;
  // -0.0698 rad matches the driver joint's "closed" command limit in
  // urdf/irb140.xacro. Override with `closed_position` at runtime.
  static constexpr double DEFAULT_CLOSED_POSITION = -0.0698;
  static constexpr double DEFAULT_JOINT_STATE_PUBLISH_RATE = 50.0;  // Hz

  // The 12 pneumatic gripper finger joints, matching the ros2_control block in
  // urdf/irb140.xacro (left finger, then right finger; the other 11 mimic the
  // left link_1_to_link_2 joint).
  static constexpr std::array<const char *, 12> GRIPPER_JOINT_NAMES = {
    "pneumatic_gripper_left_link_1_to_link_2",
    "pneumatic_gripper_left_link_2_to_link_3",
    "pneumatic_gripper_left_link_3_to_link_4",
    "pneumatic_gripper_left_link_4_to_link_5",
    "pneumatic_gripper_left_link_5_to_link_6",
    "pneumatic_gripper_left_link_6_to_link_7",
    "pneumatic_gripper_right_link_1_to_link_2",
    "pneumatic_gripper_right_link_2_to_link_3",
    "pneumatic_gripper_right_link_3_to_link_4",
    "pneumatic_gripper_right_link_4_to_link_5",
    "pneumatic_gripper_right_link_5_to_link_6",
    "pneumatic_gripper_right_link_6_to_link_7",
  };

  // --- Runtime state ---------------------------------------------------
  std::string base_url_;
  bool sim_ = false;
  bool gripper_enabled_ = false;
  double gripper_effort_percentage_ = DEFAULT_GRIPPER_EFFORT_PERCENTAGE;
  double open_position_ = DEFAULT_OPEN_POSITION;
  double closed_position_ = DEFAULT_CLOSED_POSITION;
  double joint_state_publish_rate_ = DEFAULT_JOINT_STATE_PUBLISH_RATE;
  // false = open, true = closed. Flipped only on a successful RWS send.
  std::atomic<bool> gripper_closed_{false};

  // --- Service -------------------------------------------------------
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_trigger_srv_;

  // --- Mocked joint state publishing (real robot only) ----------------
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;

  /// Service callback: opens or closes the gripper based on request.data.
  void gripper_trigger_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /// Timer callback: publishes the 12 finger joints on /joint_states, all at
  /// closed_position_ when gripper_closed_ is set, otherwise open_position_.
  void publish_gripper_joint_states();

  /**
   * POST `lvalue` to the RWS signal endpoint using HTTP digest auth.
   *
   * @param lvalue        1 to close the gripper, 0 to open it.
   * @param http_status   [out] HTTP response code returned by the controller.
   * @param error         [out] human readable error message on failure.
   * @return true on success (HTTP 200 or 204).
   */
  bool send_signal_request(int lvalue, long & http_status, std::string & error);

  /// libcurl write sink; discards the response body into a std::string.
  static std::size_t write_callback(
    void * contents, std::size_t size, std::size_t nmemb, void * userp);
};

#endif  // PNEUMATIC_GRIPPER_CONTROLLER_HPP
