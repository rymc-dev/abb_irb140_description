// Hosts a control_msgs/action/GripperCommand action server
// (gripper_controller/gripper_cmd) that opens / closes the ABB IRB140
// pneumatic gripper by setting a digital output signal on the controller via
// the Robot Web Services (RWS) REST API using HTTP digest authentication.
// Real-robot mode only -- see the class docstring in the header for why
// sim mode does nothing here.
//
//   goal position nearer closed_position_ -> lvalue 1 -> close gripper
//   goal position nearer open_position_   -> lvalue 0 -> open gripper

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <curl/curl.h>

#include "pneumatic_gripper_controller.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

PneumaticGripperController::PneumaticGripperController()
: rclcpp::Node("pneumatic_gripper_controller")
{
  // Parameters. `gripper_effort_percentage` is declared for parity with the
  // header / Python interface but has no effect: a pneumatic on/off signal
  // offers no effort control.
  this->declare_parameter<std::string>("robot_ip", DEFAULT_ROBOT_IP);
  this->declare_parameter<bool>("gripper_enabled", false);
  this->declare_parameter<double>(
    "gripper_effort_percentage", DEFAULT_GRIPPER_EFFORT_PERCENTAGE);
  this->declare_parameter<bool>("sim", false);
  this->declare_parameter<double>("open_position", DEFAULT_OPEN_POSITION);
  this->declare_parameter<double>("closed_position", DEFAULT_CLOSED_POSITION);
  this->declare_parameter<double>(
    "joint_state_publish_rate", DEFAULT_JOINT_STATE_PUBLISH_RATE);

  const std::string robot_ip = this->get_parameter("robot_ip").as_string();
  gripper_enabled_ = this->get_parameter("gripper_enabled").as_bool();
  gripper_effort_percentage_ =
    this->get_parameter("gripper_effort_percentage").as_double();
  sim_ = this->get_parameter("sim").as_bool();
  open_position_ = this->get_parameter("open_position").as_double();
  closed_position_ = this->get_parameter("closed_position").as_double();
  joint_state_publish_rate_ =
    this->get_parameter("joint_state_publish_rate").as_double();

  base_url_ = "http://" + robot_ip + "/rw/iosystem/signals/" + SIGNAL_NAME;

  // Real robot only: gz_ros2_control's own gripper_controller already serves
  // this identical action name in sim (see config/controllers.yaml), so
  // standing up a second server here would just collide with it -- and
  // sim_robot.launch.py doesn't launch this node at all as a result.
  if (!sim_) {
    gripper_action_server_ = rclcpp_action::create_server<GripperCommand>(
      this,
      GRIPPER_ACTION_NAME,
      std::bind(&PneumaticGripperController::handle_goal, this, _1, _2),
      std::bind(&PneumaticGripperController::handle_cancel, this, _1),
      std::bind(&PneumaticGripperController::handle_accepted, this, _1));

    // The sim's joint_state_broadcaster is absent on the real robot, so mock
    // the finger joint states here too.
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
    const double rate =
      joint_state_publish_rate_ > 0.0 ? joint_state_publish_rate_
                                      : DEFAULT_JOINT_STATE_PUBLISH_RATE;
    const auto period = std::chrono::duration<double>(1.0 / rate);
    joint_state_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PneumaticGripperController::publish_gripper_joint_states, this));
  }

  const std::string action_server_status =
    sim_ ? std::string("none -- gz_ros2_control's gripper_controller already serves ") +
      GRIPPER_ACTION_NAME
         : std::string(GRIPPER_ACTION_NAME);

  RCLCPP_INFO(
    this->get_logger(),
    "Gripper controller started.\n"
    "\tRobot IP: %s\n"
    "\tSignal: %s\n"
    "\tMode: %s\n"
    "\tAction server: %s\n"
    "\tFinger joint state mock: %s",
    robot_ip.c_str(), SIGNAL_NAME, sim_ ? "SIMULATION" : "REAL",
    action_server_status.c_str(),
    sim_ ? "disabled (sim broadcaster owns /joint_states)"
         : "publishing on /joint_states");
}

rclcpp_action::GoalResponse PneumaticGripperController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GripperCommand::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PneumaticGripperController::handle_cancel(
  const std::shared_ptr<GoalHandleGripperCommand> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void PneumaticGripperController::handle_accepted(
  const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
  // Runs the blocking RWS HTTP call off the node's executor thread, same as
  // any other long-running action goal.
  std::thread{
    &PneumaticGripperController::execute_gripper_goal, this, goal_handle}
    .detach();
}

void PneumaticGripperController::execute_gripper_goal(
  const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const bool close =
    std::abs(goal->command.position - closed_position_) <
    std::abs(goal->command.position - open_position_);
  const int lvalue = close ? 1 : 0;
  const std::string action_str = close ? "CLOSE" : "OPEN";

  RCLCPP_INFO(
    this->get_logger(), "Gripper goal received: %s (requested position %.4f)",
    action_str.c_str(), goal->command.position);

  auto result = std::make_shared<GripperCommand::Result>();
  long http_status = 0;
  std::string error;

  if (send_signal_request(lvalue, http_status, error)) {
    // Only now does the mocked finger joint state follow the command.
    gripper_closed_.store(close);
    result->position = close ? closed_position_ : open_position_;
    result->effort = 0.0;
    result->stalled = false;
    result->reached_goal = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Gripper %s successful", action_str.c_str());
  } else {
    result->position = gripper_closed_.load() ? closed_position_ : open_position_;
    result->effort = 0.0;
    result->stalled = false;
    result->reached_goal = false;
    goal_handle->abort(result);
    RCLCPP_ERROR(
      this->get_logger(), "Failed to %s gripper. %s", action_str.c_str(), error.c_str());
  }
}

void PneumaticGripperController::publish_gripper_joint_states()
{
  const double position =
    gripper_closed_.load() ? closed_position_ : open_position_;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->get_clock()->now();
  msg.name.assign(GRIPPER_JOINT_NAMES.begin(), GRIPPER_JOINT_NAMES.end());
  msg.position.assign(GRIPPER_JOINT_NAMES.size(), position);

  joint_state_pub_->publish(msg);
}

bool PneumaticGripperController::send_signal_request(
  int lvalue, long & http_status, std::string & error)
{
  http_status = 0;
  error.clear();

  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    error = "failed to initialise libcurl";
    return false;
  }

  const std::string url = base_url_ + "?action=set";
  const std::string post_fields = "lvalue=" + std::to_string(lvalue);
  const std::string userpwd = std::string(USERNAME) + ":" + PASSWORD;
  std::string body;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_fields.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPAUTH, static_cast<long>(CURLAUTH_DIGEST));
  curl_easy_setopt(curl, CURLOPT_USERPWD, userpwd.c_str());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(TIMEOUT));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &PneumaticGripperController::write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);

  const CURLcode rc = curl_easy_perform(curl);

  bool ok = false;
  switch (rc) {
    case CURLE_OK:
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_status);
      // 200 = OK with content, 204 = No Content (expected for SET operations).
      if (http_status == 200 || http_status == 204) {
        ok = true;
      } else {
        error = "Status code: " + std::to_string(http_status) +
          ". Response: " + body;
      }
      break;

    case CURLE_OPERATION_TIMEDOUT:
      error = "Request timeout (" + std::to_string(static_cast<long>(TIMEOUT)) +
        "s) while controlling gripper";
      break;

    case CURLE_COULDNT_CONNECT:
    case CURLE_COULDNT_RESOLVE_HOST:
      error = std::string("Failed to connect to robot: ") +
        curl_easy_strerror(rc);
      break;

    default:
      error = std::string("libcurl error: ") + curl_easy_strerror(rc);
      break;
  }

  curl_easy_cleanup(curl);
  return ok;
}

std::size_t PneumaticGripperController::write_callback(
  void * contents, std::size_t size, std::size_t nmemb, void * userp)
{
  const std::size_t total = size * nmemb;
  static_cast<std::string *>(userp)->append(
    static_cast<char *>(contents), total);
  return total;
}

int main(int argc, char ** argv)
{
  curl_global_init(CURL_GLOBAL_DEFAULT);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PneumaticGripperController>());
  rclcpp::shutdown();

  curl_global_cleanup();
  return 0;
}
