#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#ifdef HIKROBOT_CAMERA_HAS_MVS
#include "MvCameraControl.h"
#endif

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  bool connect();
  void disconnect();
  bool configure();
  void acquire();
  void reconnect();
  void publish_actual_frame_rate();
  bool set_pixel_format(const std::string & value);
  rcl_interfaces::msg::SetParametersResult set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  std::string camera_ip_;
  std::string serial_number_;
  std::string image_topic_;
  std::string actual_frame_rate_topic_;
  double exposure_us_{10000.0};
  double gain_db_{0.0};
  double frame_rate_{30.0};
  std::string pixel_format_{"BGR8"};
  int reconnect_period_ms_{1000};
  bool connected_{false};
  bool grabbing_{false};
  bool shutting_down_{false};
  unsigned int consecutive_capture_errors_{0};
  unsigned int frames_in_rate_window_{0};
  std::chrono::steady_clock::time_point rate_window_start_;
  std::mutex mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actual_frame_rate_pub_;
  rclcpp::TimerBase::SharedPtr actual_frame_rate_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

#ifdef HIKROBOT_CAMERA_HAS_MVS
  void * handle_{nullptr};
#endif
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
