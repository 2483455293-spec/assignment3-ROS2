#include "hikrobot_camera/camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hikrobot_camera
{
namespace
{

const char * mvs_error_description(int error)
{
#ifdef HIKROBOT_CAMERA_HAS_MVS
  switch (static_cast<unsigned int>(error)) {
    case MV_E_ACCESS_DENIED:
      return "device access denied";
    case MV_E_RESOURCE_IN_USE:
      return "camera resource is already in use";
    case MV_E_DEV_OFFLINE:
      return "camera is offline";
    case MV_E_USB_READ:
      return "USB read error";
    case MV_E_USB_WRITE:
      return "USB write error";
    case MV_E_USB_DEVICE:
      return "USB device error";
    case MV_E_USB_DRIVER:
      return "USB driver error";
    default:
      return "see the MVS SDK error definitions";
  }
#else
  (void)error;
  return "MVS SDK unavailable";
#endif
}

#ifdef HIKROBOT_CAMERA_HAS_MVS
unsigned int pixel_format_value(const std::string & value)
{
  if (value == "MONO8") {
    return PixelType_Gvsp_Mono8;
  }
  return value == "RGB8" ? PixelType_Gvsp_RGB8_Packed : PixelType_Gvsp_BGR8_Packed;
}

struct FrameBufferGuard
{
  void * handle;
  MV_FRAME_OUT * frame;

  ~FrameBufferGuard()
  {
    if (frame->pBufAddr) {
      MV_CC_FreeImageBuffer(handle, frame);
    }
  }
};
#endif

}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  camera_ip_ = declare_parameter("camera_ip", "");
  serial_number_ = declare_parameter("serial_number", "");
  image_topic_ = declare_parameter("image_topic", "/image_raw");
  image_qos_reliability_ =
    declare_parameter("image_qos_reliability", image_qos_reliability_);
  actual_frame_rate_topic_ =
    declare_parameter("actual_frame_rate_topic", "/camera/actual_fps");
  exposure_us_ = declare_parameter("exposure_us", exposure_us_);
  gain_db_ = declare_parameter("gain_db", gain_db_);
  frame_rate_ = declare_parameter("frame_rate", frame_rate_);
  pixel_format_ = declare_parameter("pixel_format", pixel_format_);
  reconnect_period_ms_ = declare_parameter("reconnect_period_ms", reconnect_period_ms_);
  if (exposure_us_ < 1.0 || exposure_us_ > 10000000.0 ||
    gain_db_ < 0.0 || gain_db_ > 100.0 || frame_rate_ <= 0.0 || frame_rate_ > 1000.0 ||
    (pixel_format_ != "BGR8" && pixel_format_ != "RGB8" && pixel_format_ != "MONO8") ||
    (image_qos_reliability_ != "reliable" && image_qos_reliability_ != "best_effort"))
  {
    throw std::invalid_argument(
      "Invalid camera parameters: exposure_us [1,10000000], gain_db [0,100], "
      "frame_rate (0,1000], pixel_format BGR8/RGB8/MONO8, "
      "image_qos_reliability reliable/best_effort");
  }

  // A RELIABLE publisher is matched by both RELIABLE and BEST_EFFORT subscribers, so the
  // default also works with RViz2 (whose Image display subscribes with RELIABLE). A
  // BEST_EFFORT publisher is only matched by BEST_EFFORT subscribers.
  rclcpp::QoS image_qos = rclcpp::QoS(rclcpp::KeepLast(5)).durability_volatile();
  if (image_qos_reliability_ == "best_effort") {
    image_qos.best_effort();
  } else {
    image_qos.reliable();
  }
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, image_qos);
  RCLCPP_INFO(
    get_logger(), "Publishing images on %s with %s reliability",
    image_topic_.c_str(), image_qos_reliability_.c_str());
  actual_frame_rate_pub_ =
    create_publisher<std_msgs::msg::Float64>(actual_frame_rate_topic_, 10);
  parameter_callback_ = add_on_set_parameters_callback(
    std::bind(&CameraNode::set_parameters, this, std::placeholders::_1));
  reconnect_timer_ = create_wall_timer(
    std::chrono::milliseconds(std::max(100, reconnect_period_ms_)),
    std::bind(&CameraNode::reconnect, this));
  actual_frame_rate_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&CameraNode::publish_actual_frame_rate, this));
  capture_timer_ = create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&CameraNode::acquire, this));
  rate_window_start_ = std::chrono::steady_clock::now();
  reconnect();
}

CameraNode::~CameraNode()
{
  shutting_down_ = true;
  std::lock_guard<std::mutex> lock(mutex_);
  disconnect();
}

void CameraNode::reconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_ && !shutting_down_) {
    connected_ = connect();
  }
}

void CameraNode::publish_actual_frame_rate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now_steady = std::chrono::steady_clock::now();
  const auto elapsed = now_steady - rate_window_start_;
  if (elapsed <= std::chrono::steady_clock::duration::zero()) {
    return;
  }
  std_msgs::msg::Float64 actual_rate;
  actual_rate.data = static_cast<double>(frames_in_rate_window_) /
    std::chrono::duration<double>(elapsed).count();
  actual_frame_rate_pub_->publish(actual_rate);
  frames_in_rate_window_ = 0;
  rate_window_start_ = now_steady;
}

bool CameraNode::connect()
{
#ifndef HIKROBOT_CAMERA_HAS_MVS
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 10000,
    "MVS SDK is not installed. Set MVS_ROOT or MVCAM_SDK_PATH and rebuild.");
  return false;
#else
  MV_CC_DEVICE_INFO_LIST devices{};
  int rc = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
  if (rc != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_EnumDevices failed: 0x%08x", rc);
    return false;
  }
  RCLCPP_INFO(get_logger(), "MVS discovered %u device(s)", devices.nDeviceNum);
  MV_CC_DEVICE_INFO * selected = nullptr;
  for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
    auto * info = devices.pDeviceInfo[i];
    if (!info) continue;
    std::string serial;
    std::string ip;
    if (info->nTLayerType == MV_GIGE_DEVICE) {
      const auto & gige = info->SpecialInfo.stGigEInfo;
      serial = std::string(
        reinterpret_cast<const char *>(gige.chSerialNumber),
        strnlen(reinterpret_cast<const char *>(gige.chSerialNumber), sizeof(gige.chSerialNumber)));
      std::ostringstream address;
      address << ((gige.nCurrentIp >> 24) & 0xff) << '.'
              << ((gige.nCurrentIp >> 16) & 0xff) << '.'
              << ((gige.nCurrentIp >> 8) & 0xff) << '.' << (gige.nCurrentIp & 0xff);
      ip = address.str();
    } else {
      const auto * serial_data =
        reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
      serial = std::string(
        serial_data, strnlen(serial_data, sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber)));
    }
    const bool matches_ip = camera_ip_.empty() || camera_ip_ == ip;
    const bool matches_serial = serial_number_.empty() || serial_number_ == serial;
    if (matches_ip && matches_serial)
    {
      if (selected) {
        RCLCPP_ERROR(get_logger(), "Camera selector is ambiguous; specify camera_ip or serial_number");
        return false;
      }
      selected = info;
    }
  }
  if (!selected) {
    RCLCPP_WARN(get_logger(), "No camera matched camera_ip='%s', serial_number='%s'",
      camera_ip_.c_str(), serial_number_.c_str());
    return false;
  }
  void * new_handle = nullptr;
  rc = MV_CC_CreateHandle(&new_handle, selected);
  if (rc != MV_OK) {
    RCLCPP_ERROR(get_logger(), "MV_CC_CreateHandle failed: 0x%08x", rc);
  }
  if (rc == MV_OK) {
    rc = MV_CC_OpenDevice(new_handle);
    if (rc != MV_OK) {
      RCLCPP_ERROR(
        get_logger(),
        "MV_CC_OpenDevice failed: 0x%08x (%s)", static_cast<unsigned int>(rc),
        mvs_error_description(rc));
    }
  }
  if (rc == MV_OK) {
    handle_ = new_handle;
    if (!configure()) {
      RCLCPP_ERROR(get_logger(), "Camera feature configuration failed");
      disconnect();
      return false;
    }
  }
  if (rc == MV_OK) {
    rc = MV_CC_StartGrabbing(handle_);
    if (rc != MV_OK) {
      RCLCPP_ERROR(
        get_logger(), "MV_CC_StartGrabbing failed: 0x%08x (%s)",
        static_cast<unsigned int>(rc), mvs_error_description(rc));
    } else {
      grabbing_ = true;
    }
  }
  if (rc != MV_OK) {
    RCLCPP_ERROR(
      get_logger(), "Camera connection/configuration failed: 0x%08x (%s)",
      static_cast<unsigned int>(rc), mvs_error_description(rc));
    if (handle_) {
      disconnect();
    } else if (new_handle) {
      MV_CC_CloseDevice(new_handle);
      MV_CC_DestroyHandle(new_handle);
    }
    return false;
  }
  consecutive_capture_errors_ = 0;
  frames_in_rate_window_ = 0;
  rate_window_start_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "Camera connected");
  return true;
#endif
}

void CameraNode::disconnect()
{
#ifdef HIKROBOT_CAMERA_HAS_MVS
  if (handle_) {
    if (grabbing_) {
      MV_CC_StopGrabbing(handle_);
      grabbing_ = false;
    }
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
#endif
  connected_ = false;
}

bool CameraNode::configure()
{
#ifndef HIKROBOT_CAMERA_HAS_MVS
  return false;
#else
  int rc = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
  if (rc != MV_OK) {
    RCLCPP_ERROR(get_logger(), "Setting TriggerMode=Off failed: 0x%08x", rc);
  }
  if (rc == MV_OK) {
    rc = MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    if (rc != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Disabling automatic exposure failed: 0x%08x", rc);
    }
  }
  if (rc == MV_OK) {
    rc = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    if (rc != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Disabling automatic gain failed: 0x%08x", rc);
    }
  }
  if (rc == MV_OK) {
    rc = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    if (rc != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Enabling AcquisitionFrameRate failed: 0x%08x",
        static_cast<unsigned int>(rc));
    }
  }
  if (rc == MV_OK) {
    rc = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate_));
    if (rc != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Setting AcquisitionFrameRate failed: 0x%08x", rc);
    }
  }
  if (rc == MV_OK) {
    rc = MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_us_));
    if (rc != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Setting ExposureTime failed: 0x%08x", rc);
    }
  }
  if (rc == MV_OK) {
    rc = MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_db_));
    if (rc != MV_OK) {
      RCLCPP_ERROR(get_logger(), "Setting Gain failed: 0x%08x", rc);
    }
  }
  if (rc == MV_OK && !set_pixel_format(pixel_format_)) {
    return false;
  }
  return rc == MV_OK;
#endif
}

bool CameraNode::set_pixel_format(const std::string & value)
{
#ifdef HIKROBOT_CAMERA_HAS_MVS
  MVCC_ENUMVALUE supported_formats{};
  int rc = MV_CC_GetEnumValue(handle_, "PixelFormat", &supported_formats);
  if (rc != MV_OK) {
    RCLCPP_ERROR(get_logger(), "Reading supported PixelFormat values failed: 0x%08x",
      static_cast<unsigned int>(rc));
    return false;
  }

  const auto requested = pixel_format_value(value);
  const auto supported_count = std::min(
    supported_formats.nSupportedNum,
    static_cast<unsigned int>(sizeof(supported_formats.nSupportValue) /
    sizeof(supported_formats.nSupportValue[0])));
  const bool is_supported = std::find(
    supported_formats.nSupportValue,
    supported_formats.nSupportValue + supported_count,
    requested) != supported_formats.nSupportValue + supported_count;
  if (!is_supported) {
    RCLCPP_ERROR(get_logger(), "Camera does not support requested PixelFormat=%s",
      value.c_str());
    return false;
  }
  rc = MV_CC_SetEnumValue(handle_, "PixelFormat", requested);
  if (rc != MV_OK) {
    RCLCPP_ERROR(get_logger(), "Setting PixelFormat=%s failed: 0x%08x",
      value.c_str(), static_cast<unsigned int>(rc));
    return false;
  }
  return true;
#else
  (void)value;
  return false;
#endif
}

void CameraNode::acquire()
{
  std::lock_guard<std::mutex> lock(mutex_);
#ifdef HIKROBOT_CAMERA_HAS_MVS
  if (!connected_ || !handle_) return;
  if (!MV_CC_IsDeviceConnected(handle_)) {
    RCLCPP_WARN(get_logger(), "Camera disconnected; will retry connection");
    disconnect();
    return;
  }
  MV_FRAME_OUT frame{};
  const int rc = MV_CC_GetImageBuffer(handle_, &frame, 20);
  if (rc != MV_OK) {
    if (static_cast<unsigned int>(rc) == MV_E_NODATA) {
      return;
    }
    ++consecutive_capture_errors_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "MV_CC_GetImageBuffer failed: 0x%08x (%s); consecutive failures: %u",
      static_cast<unsigned int>(rc), mvs_error_description(rc), consecutive_capture_errors_);
    if (consecutive_capture_errors_ >= 3 || !MV_CC_IsDeviceConnected(handle_)) {
      RCLCPP_WARN(get_logger(), "Camera capture failed repeatedly; disconnecting for retry");
      disconnect();
    }
    return;
  }
  consecutive_capture_errors_ = 0;
  FrameBufferGuard frame_guard{handle_, &frame};
  if (!frame.pBufAddr || frame.stFrameInfo.nWidth == 0 || frame.stFrameInfo.nHeight == 0) {
    RCLCPP_ERROR(get_logger(), "MVS returned an invalid image buffer");
    return;
  }

  auto msg = sensor_msgs::msg::Image();
  msg.header.stamp = now();
  msg.header.frame_id = "camera";
  msg.width = frame.stFrameInfo.nWidth;
  msg.height = frame.stFrameInfo.nHeight;
  msg.encoding = pixel_format_ == "MONO8" ? "mono8" :
    (pixel_format_ == "RGB8" ? "rgb8" : "bgr8");
  msg.is_bigendian = false;
  const std::size_t channels = pixel_format_ == "MONO8" ? 1U : 3U;
  const std::size_t min_step = static_cast<std::size_t>(msg.width) * channels;
  const std::size_t frame_length = frame.stFrameInfo.nFrameLen;
  if (frame_length % msg.height != 0 || frame_length / msg.height < min_step ||
    frame_length / msg.height > std::numeric_limits<std::uint32_t>::max())
  {
    RCLCPP_ERROR(
      get_logger(), "Invalid image buffer size: %u bytes for %ux%u %s",
      frame.stFrameInfo.nFrameLen, msg.width, msg.height, msg.encoding.c_str());
    return;
  }
  msg.step = static_cast<std::uint32_t>(frame_length / msg.height);
  msg.data.assign(frame.pBufAddr, frame.pBufAddr + frame.stFrameInfo.nFrameLen);
  image_pub_->publish(std::move(msg));

  ++frames_in_rate_window_;
#endif
}

rcl_interfaces::msg::SetParametersResult CameraNode::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  std::lock_guard<std::mutex> lock(mutex_);

  std::optional<double> requested_exposure;
  std::optional<double> requested_gain;
  std::optional<double> requested_frame_rate;
  std::optional<std::string> requested_pixel_format;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "exposure_us") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "exposure_us must be a double";
        return result;
      }
      const auto value = parameter.as_double();
      if (!std::isfinite(value) || value < 1.0 || value > 10000000.0) {
        result.successful = false;
        result.reason = "exposure_us must be in [1, 10000000]";
        return result;
      }
      requested_exposure = value;
    } else if (parameter.get_name() == "gain_db") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "gain_db must be a double";
        return result;
      }
      const auto value = parameter.as_double();
      if (!std::isfinite(value) || value < 0.0 || value > 100.0) {
        result.successful = false;
        result.reason = "gain_db must be in [0, 100]";
        return result;
      }
      requested_gain = value;
    } else if (parameter.get_name() == "frame_rate") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "frame_rate must be a double";
        return result;
      }
      const auto value = parameter.as_double();
      if (!std::isfinite(value) || value <= 0.0 || value > 1000.0) {
        result.successful = false;
        result.reason = "frame_rate must be in (0, 1000]";
        return result;
      }
      requested_frame_rate = value;
    } else if (parameter.get_name() == "pixel_format") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        result.successful = false;
        result.reason = "pixel_format must be a string";
        return result;
      }
      const auto value = parameter.as_string();
      if (value != "BGR8" && value != "RGB8" && value != "MONO8") {
        result.successful = false;
        result.reason = "pixel_format must be BGR8, RGB8, or MONO8";
        return result;
      }
      requested_pixel_format = value;
    } else if (parameter.get_name() == "image_qos_reliability") {
      result.successful = false;
      result.reason = "image_qos_reliability is only applied at startup";
      return result;
    }
  }

  if (!connected_) {
    if (requested_exposure) exposure_us_ = *requested_exposure;
    if (requested_gain) gain_db_ = *requested_gain;
    if (requested_frame_rate) frame_rate_ = *requested_frame_rate;
    if (requested_pixel_format) pixel_format_ = *requested_pixel_format;
    return result;
  }

#ifdef HIKROBOT_CAMERA_HAS_MVS
  const double old_exposure = exposure_us_;
  const double old_gain = gain_db_;
  const double old_frame_rate = frame_rate_;
  const std::string old_pixel_format = pixel_format_;
  const bool pixel_format_changed =
    requested_pixel_format && *requested_pixel_format != pixel_format_;
  bool stopped_for_pixel_change = false;
  if (pixel_format_changed) {
    const int stop_rc = MV_CC_StopGrabbing(handle_);
    if (stop_rc != MV_OK) {
      result.successful = false;
      result.reason = "Could not stop image acquisition before changing pixel_format";
      return result;
    }
    grabbing_ = false;
    stopped_for_pixel_change = true;
  }

  bool applied = true;
  std::string failure_reason;
  if (requested_exposure) {
    int rc = MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    if (rc == MV_OK) {
      rc = MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(*requested_exposure));
    }
    if (rc != MV_OK) {
      applied = false;
      failure_reason = "Camera rejected exposure_us";
    }
  }
  if (applied && requested_gain) {
    int rc = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    if (rc == MV_OK) {
      rc = MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(*requested_gain));
    }
    if (rc != MV_OK) {
      applied = false;
      failure_reason = "Camera rejected gain_db";
    }
  }
  if (applied && requested_frame_rate &&
    (MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true) != MV_OK ||
    MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
    static_cast<float>(*requested_frame_rate)) != MV_OK))
  {
    applied = false;
    failure_reason = "Camera rejected frame_rate";
  }
  if (applied && pixel_format_changed && !set_pixel_format(*requested_pixel_format)) {
    applied = false;
    failure_reason = "Camera rejected pixel_format or does not support it";
  }

  if (!applied) {
    bool rollback_ok = true;
    if (requested_exposure) {
      rollback_ok = MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF) ==
        MV_OK && MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(old_exposure)) ==
        MV_OK && rollback_ok;
    }
    if (requested_gain) {
      rollback_ok = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF) == MV_OK &&
        MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(old_gain)) == MV_OK &&
        rollback_ok;
    }
    if (requested_frame_rate) {
      rollback_ok = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true) == MV_OK &&
        MV_CC_SetFloatValue(
        handle_, "AcquisitionFrameRate", static_cast<float>(old_frame_rate)) == MV_OK &&
        rollback_ok;
    }
    if (pixel_format_changed) {
      rollback_ok = set_pixel_format(old_pixel_format) && rollback_ok;
    }
    if (stopped_for_pixel_change) {
      const int start_rc = MV_CC_StartGrabbing(handle_);
      grabbing_ = start_rc == MV_OK;
      rollback_ok = rollback_ok && grabbing_;
    }
    if (!rollback_ok) {
      RCLCPP_ERROR(get_logger(), "Parameter update rollback failed; reconnecting camera");
      disconnect();
    }
    result.successful = false;
    result.reason = failure_reason;
    return result;
  }

  if (stopped_for_pixel_change) {
    const int start_rc = MV_CC_StartGrabbing(handle_);
    if (start_rc != MV_OK) {
      result.successful = false;
      result.reason = "Pixel format changed but image acquisition could not be restarted";
      disconnect();
      return result;
    }
    grabbing_ = true;
  }
#endif

  if (requested_exposure) exposure_us_ = *requested_exposure;
  if (requested_gain) gain_db_ = *requested_gain;
  if (requested_frame_rate) frame_rate_ = *requested_frame_rate;
  if (requested_pixel_format) pixel_format_ = *requested_pixel_format;
  return result;
}

} 
