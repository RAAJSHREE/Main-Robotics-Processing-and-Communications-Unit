#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "s1_interfaces/msg/heartbeat.hpp"
#include "s1_interfaces/msg/log_entry.hpp"
#include "s1_interfaces/msg/s1_command.hpp"

using namespace std::chrono_literals;
using s1_interfaces::msg::Heartbeat;
using s1_interfaces::msg::LogEntry;
using s1_interfaces::msg::S1Command;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode()
  : Node("driver_node")
  {
    this->declare_parameter<std::string>("driver_name", "p2_driver");
    driver_name_ = this->get_parameter("driver_name").as_string();

    hb_pub_ = this->create_publisher<Heartbeat>("/s1/heartbeat/raw", 10);
    log_pub_ = this->create_publisher<LogEntry>("/s1/logs/raw", 10);

    // Subscribe to driver-specific command topic from subsystem
    std::string cmd_topic = "/" + driver_name_ + "/cmd";
    cmd_sub_ = this->create_subscription<S1Command>(
      cmd_topic, 10,
      std::bind(&DriverNode::command_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms, std::bind(&DriverNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Driver node '%s' started. Listening on: %s", 
                driver_name_.c_str(), cmd_topic.c_str());
  }

private:
  void command_callback(const S1Command::SharedPtr msg)
  {
    // Handle commands from subsystem
    std::string action = msg->action;
    
    // DEBUG: Log every command received
    RCLCPP_INFO(get_logger(), "[%s] Received command: '%s'", driver_name_.c_str(), action.c_str());
    
    if (estop_active_) {
      if (action == "clear_estop") {
        estop_active_ = false;
        log_message("INFO", "🟢 E-STOP cleared - hardware operational");
        RCLCPP_WARN(get_logger(), "[%s] E-STOP FLAG CLEARED (estop_active_=false)", driver_name_.c_str());
      } else {
        log_message("WARN", "❌ Command '" + action + "' blocked - E-STOP active");
      }
      return;
    }

    if (action == "estop") {
      estop_active_ = true;
      log_message("CRITICAL", "🔴 HARDWARE E-STOP ACTIVATED - ALL MOTION STOPPED");
      RCLCPP_ERROR(get_logger(), "[%s] E-STOP FLAG SET (estop_active_=true)", driver_name_.c_str());
      // In real hardware: emergency_stop_motors();
      return;
    }

    // Execute hardware commands based on driver type
    if (driver_name_ == "p2_driver") {
      handle_p2_command(action, msg->params);
    } else if (driver_name_ == "p3_driver") {
      handle_p3_command(action, msg->params);
    } else if (driver_name_ == "p4_driver") {
      handle_p4_command(action, msg->params);
    }
  }

  void handle_p2_command(const std::string& action, const std::vector<std::string>& params)
  {
    // P2: Perception hardware (cameras, LiDAR via CANopen)
    if (action == "scan_area") {
      log_message("INFO", "📷 P2: Activating cameras and LiDAR scan");
      // Real: trigger_camera_capture(); start_lidar_scan();
    } else if (action == "calibrate_sensors") {
      log_message("INFO", "🎯 P2: Calibrating perception sensors");
      // Real: run_camera_calibration(); calibrate_lidar();
    } else if (action == "track_target") {
      std::string target = params.empty() ? "unknown" : params[0];
      log_message("INFO", "🎯 P2: Tracking target: " + target);
      // Real: enable_object_tracking(target);
    } else {
      log_message("WARN", "⚠️ P2: Unknown command: " + action);
    }
  }

  void handle_p3_command(const std::string& action, const std::vector<std::string>& params)
  {
    // P3: Motion hardware (motor controllers via Ethernet)
    if (action == "move_forward") {
      log_message("INFO", "⬆️ P3: Motors FORWARD");
      // Real: set_motor_speeds(left: 1.0, right: 1.0);
    } else if (action == "move_back") {
      log_message("INFO", "⬇️ P3: Motors BACKWARD");
      // Real: set_motor_speeds(left: -1.0, right: -1.0);
    } else if (action == "turn_left") {
      log_message("INFO", "⬅️ P3: Motors TURN LEFT");
      // Real: set_motor_speeds(left: -0.5, right: 0.5);
    } else if (action == "turn_right") {
      log_message("INFO", "➡️ P3: Motors TURN RIGHT");
      // Real: set_motor_speeds(left: 0.5, right: -0.5);
    } else if (action == "stop") {
      log_message("INFO", "🛑 P3: Motors STOP");
      // Real: set_motor_speeds(left: 0, right: 0);
    } else {
      log_message("WARN", "⚠️ P3: Unknown command: " + action);
    }
  }

  void handle_p4_command(const std::string& action, const std::vector<std::string>& params)
  {
    // P4: Actuation hardware (LEDs, buzzer, gripper via LVDS/Optical)
    if (action == "led_on") {
      log_message("INFO", "💡 P4: LED ON");
      // Real: gpio_set_led(true);
    } else if (action == "led_off") {
      log_message("INFO", "🔲 P4: LED OFF");
      // Real: gpio_set_led(false);
    } else if (action == "beep") {
      log_message("INFO", "🔊 P4: BEEP!");
      // Real: activate_buzzer(duration_ms: 200);
    } else if (action == "gripper_open") {
      log_message("INFO", "🤏 P4: Gripper OPEN");
      // Real: set_servo_angle(gripper_servo, 0);
    } else if (action == "gripper_close") {
      log_message("INFO", "✊ P4: Gripper CLOSE");
      // Real: set_servo_angle(gripper_servo, 90);
    } else {
      log_message("WARN", "⚠️ P4: Unknown command: " + action);
    }
  }

  void timer_callback()
  {
    auto now = this->now();

    Heartbeat hb;
    hb.node_name = driver_name_;
    hb.stamp = now;
    hb.status = estop_active_ ? "E-STOP" : "OK";
    hb.details = estop_active_ ? "Hardware E-STOP active" : "Hardware operational";
    hb_pub_->publish(hb);

    if (++counter_ % 30 == 0) {
      LogEntry log;
      log.stamp = now;
      log.level = "INFO";
      log.source_node = driver_name_;
      log.message = "Hardware driver operational";
      log_pub_->publish(log);
    }
  }

  void log_message(const std::string& level, const std::string& message)
  {
    LogEntry log;
    log.stamp = this->now();
    log.level = level;
    log.source_node = driver_name_;
    log.message = message;
    log_pub_->publish(log);

    if (level == "CRITICAL" || level == "ERROR") {
      RCLCPP_ERROR(get_logger(), "[%s] %s", driver_name_.c_str(), message.c_str());
    } else if (level == "WARN") {
      RCLCPP_WARN(get_logger(), "[%s] %s", driver_name_.c_str(), message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "[%s] %s", driver_name_.c_str(), message.c_str());
    }
  }

  std::string driver_name_;
  rclcpp::Publisher<Heartbeat>::SharedPtr hb_pub_;
  rclcpp::Publisher<LogEntry>::SharedPtr log_pub_;
  rclcpp::Subscription<S1Command>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_ = 0;
  bool estop_active_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverNode>());
  rclcpp::shutdown();
  return 0;
}
