// Provides a std_srvs/srv/SetBool service that opens / closes the ABB IRB140
// pneumatic gripper by setting a digital output signal on the controller via
// the Robot Web Services (RWS) REST API using HTTP digest authentication.
//
//   data = true  -> lvalue 1 -> close gripper
//   data = false -> lvalue 0 -> open gripper

#include <chrono>
#include <functional>
#include <memory>
#include <string>
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

  gripper_trigger_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "gripper_trigger",
    std::bind(&PneumaticGripperController::gripper_trigger_cb, this, _1, _2));

  if (sim_) {
    gripper_action_client_ =
      rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      this, GRIPPER_ACTION_NAME);
  }

  // On the real robot the sim's joint_state_broadcaster is absent, so mock the
  // finger joint states here. In sim we stay out of the way of the broadcaster.
  if (!sim_) {
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

  RCLCPP_INFO(
    this->get_logger(),
    "Gripper trigger service started.\n"
    "\tRobot IP: %s\n"
    "\tSignal: %s\n"
    "\tService: /gripper_trigger\n"
    "\tMode: %s\n"
    "\tFinger joint state mock: %s\n"
    "\tRequest format: SetBool (data=True for close, data=False for open)",
    robot_ip.c_str(), SIGNAL_NAME, sim_ ? "SIMULATION" : "REAL",
    sim_ ? "disabled (sim broadcaster owns /joint_states)"
         : "publishing on /joint_states");
}

void PneumaticGripperController::gripper_trigger_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const int lvalue = request->data ? 1 : 0;
  const std::string action_str = request->data ? "CLOSE" : "OPEN";

  RCLCPP_INFO(
    this->get_logger(), "Gripper trigger requested: %s", action_str.c_str());

  if (sim_) {
    send_sim_gripper_goal(request->data, response);
    return;
  }

  long http_status = 0;
  std::string error;

  if (send_signal_request(lvalue, http_status, error)) {
    // Only now does the mocked finger joint state follow the command.
    gripper_closed_.store(request->data);
    response->success = true;
    response->message = "Gripper " + action_str + " command sent successfully";
    RCLCPP_INFO(
      this->get_logger(), "Gripper %s successful", action_str.c_str());
  } else {
    response->success = false;
    response->message = "Failed to " + action_str + " gripper. " + error;
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

void PneumaticGripperController::send_sim_gripper_goal(
  bool close, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const std::string action_str = close ? "CLOSE" : "OPEN";

  if (!gripper_action_client_->wait_for_action_server(
      std::chrono::duration<double>(GRIPPER_ACTION_SERVER_WAIT)))
  {
    response->success = false;
    response->message =
      "[SIM] Gripper " + action_str + " failed: gripper_controller/gripper_cmd "
      "action server not available -- is sim_robot.launch.py's "
      "gripper_controller_spawner running?";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  control_msgs::action::GripperCommand::Goal goal;
  goal.command.position = close ? closed_position_ : open_position_;
  // 0.0 defers to gripper_controller's own max_effort (config/controllers.yaml).
  goal.command.max_effort = 0.0;

  rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions options;
  options.goal_response_callback =
    [this, action_str](
    const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::SharedPtr
    & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(
          this->get_logger(), "[SIM] Gripper %s goal was rejected", action_str.c_str());
      } else {
        RCLCPP_INFO(
          this->get_logger(), "[SIM] Gripper %s goal accepted", action_str.c_str());
      }
    };
  options.result_callback =
    [this, action_str](
    const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult
    & result) {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(
          this->get_logger(), "[SIM] Gripper %s goal did not succeed (code %d)",
          action_str.c_str(), static_cast<int>(result.code));
      }
    };

  gripper_action_client_->async_send_goal(goal, options);

  // Fire-and-forget: report success once the goal has been dispatched, without
  // waiting for it to be accepted or for the fingers to finish moving.
  response->success = true;
  response->message = "[SIM] Gripper " + action_str + " goal sent";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
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
