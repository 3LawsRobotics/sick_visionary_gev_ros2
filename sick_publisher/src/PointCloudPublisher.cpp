// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "PointCloudPublisher.hpp"

#include <stdint.h>

#include <functional>
#include <string>

#ifdef ROS2_HUMBLE
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.hpp>
#endif
#include <opencv2/core.hpp>
#include <opencv2/core/operations.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include "base/Logging.hpp"
#include "utils/PointCloudUtils.hpp"

namespace rclcpp
{
class NodeOptions;
}

namespace sick
{

using sensor_msgs::msg::Image;

PointCloudPublisher::PointCloudPublisher(const rclcpp::NodeOptions& options)
  : Node("pointcloud_publisher", options)
{
  setupParameterHandling();
  m_cameraFrame = "camera_frame_" + std::to_string(m_serialNumber);

  // Callbackgroups
  m_subscriptionsCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_pointCloudCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_camInfoSubCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Subscriptions
  auto subCbOpts = rclcpp::SubscriptionOptions();
  subCbOpts.callback_group = m_subscriptionsCallbackGroup;

  const auto sensorQos = rclcpp::SensorDataQoS();
  const auto qosProfile = sensorQos.get_rmw_qos_profile();

  // clang-format off
  auto topic = [this](const std::string& suffix) { return m_namespace + "/" + "cam_" + std::to_string(m_serialNumber) + suffix; };
  // clang-format on

  std::string intensityTopic;
  if (m_namespace.find("visionary_t_mini") != std::string::npos)
  {
    intensityTopic = "/grayscale";
  }
  else if (m_namespace.find("visionary_b_two") != std::string::npos ||
           m_namespace.find("visionary_s") != std::string::npos)
  {
    intensityTopic = "/rgb";
  }

  m_intensityImgSub.subscribe(this, topic(intensityTopic), qosProfile, subCbOpts);
  m_depthImgSub.subscribe(this, topic("/depth"), qosProfile, subCbOpts);

  auto camInfoSubCbOpts = rclcpp::SubscriptionOptions();
  camInfoSubCbOpts.callback_group = m_camInfoSubCallbackGroup;

  m_camInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic("/camera_info"),
    sensorQos,
    [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { this->onCameraInfo(msg); },
    camInfoSubCbOpts);

  // TimeSynchronizer
  m_timeSync.reset(new message_filters::TimeSynchronizer<Image, Image>(m_intensityImgSub, m_depthImgSub, 5));
  m_timeSync->registerCallback(
    std::bind(&PointCloudPublisher::onPublishPointcloud, this, std::placeholders::_1, std::placeholders::_2));

  // Publisher
  auto pcCbOpts = rclcpp::PublisherOptions();
  pcCbOpts.callback_group = m_pointCloudCallbackGroup;
  m_pointCloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // QoS profile for pointcloud data
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  m_pointCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>(topic("/pointcloud"), qos, pcCbOpts);
}

void PointCloudPublisher::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camInfo)
{
  // TODO(xfealal/ES-366): Change this to service
  if (!m_cameraIntrinsicsFilled)
  {
    m_cameraIntrinsics.focalLength = camInfo->k[0];
    const float effectiveFocalLength = camInfo->k[4];

    m_cameraIntrinsics.principalPointU = camInfo->k[2];
    m_cameraIntrinsics.principalPointV = camInfo->k[5];

    m_cameraIntrinsics.scaleC = 1.; // Default for most cameras
    m_cameraIntrinsics.offset = 0.; // Default for most cameras

    // Initialize cache values for x,y coordinates
    const uint32_t height = camInfo->height;
    const uint32_t width = camInfo->width;

    const float invFocalLength = 1.0F / m_cameraIntrinsics.focalLength;
    const float invEffectiveFocalLength = 1.0F / effectiveFocalLength;

    const float principalPointU = m_cameraIntrinsics.principalPointU;
    const float principalPointV = m_cameraIntrinsics.principalPointV;

    m_cachedX.resize(width * height);
    m_cachedY.resize(width * height);

    for (uint32_t v = 0; v < height; ++v)
    {
      for (uint32_t u = 0; u < width; ++u)
      {
        uint32_t idx = v * width + u;
        m_cachedX[idx] = (u - principalPointU) * invFocalLength;
        m_cachedY[idx] = (v - principalPointV) * invEffectiveFocalLength;
      }
    }

    m_cameraIntrinsicsFilled = true;
  }
};

void PointCloudPublisher::onPublishPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr& intensityImg,
                                              const sensor_msgs::msg::Image::ConstSharedPtr& depthImg)
{
  if (!m_publishPointcloud)
    return;

  // Convert ROS images to OpenCV images
  const cv::Mat depthImage16 = cv_bridge::toCvShare(depthImg)->image;
  // ROS expects the depth data to be given in meters, whereas the Visionary camera
  // outputs in millimeters. Hence, we adjust the depth data values accordingly
  cv::Mat depthImage32;
  depthImage16.convertTo(depthImage32, CV_32FC1);
  depthImage32 /= 1000.0;

  // Fill PointCloud2 message
  const auto height = static_cast<uint32_t>(depthImage32.rows);
  const auto width = static_cast<uint32_t>(depthImage32.cols);
  m_pointCloudMsg->height = height;
  m_pointCloudMsg->width = width;
  m_pointCloudMsg->is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(*m_pointCloudMsg);
  const float* depthPtr = depthImage32.ptr<float>();

  if (intensityImg->encoding == "bgr8")
  {
    rgbToPointcloud(intensityImg,
                    depthPtr,
                    height,
                    width,
                    m_cachedX,
                    m_cachedY,
                    m_cameraIntrinsics.scaleC,
                    m_cameraIntrinsics.offset,
                    *m_pointCloudMsg,
                    modifier);
  }
  else
  {
    grayscaleToPointcloud(intensityImg,
                          depthPtr,
                          height,
                          width,
                          m_cachedX,
                          m_cachedY,
                          m_cameraIntrinsics.scaleC,
                          m_cameraIntrinsics.offset,
                          *m_pointCloudMsg,
                          modifier);
  }

  m_pointCloudMsg->header = intensityImg->header;
  m_pointCloudMsg->header.frame_id = m_cameraFrame;
  m_pointCloudPublisher->publish(*m_pointCloudMsg);
}

void PointCloudPublisher::setupParameterHandling()
{
  declare_parameter<int>("serial_number", m_serialNumber);
  declare_parameter<bool>("publish_pointcloud", m_publishPointcloud);

  auto parameterChangeCallback = [this](const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params)
    {
      if (param.get_name() == "serial_number")
      {
        m_serialNumber = param.as_int();
      }

      if (param.get_name() == "publish_pointcloud" && param.as_bool() != m_publishPointcloud)
      {
        m_publishPointcloud = param.as_bool();
        if (m_publishPointcloud)
        {
          infoLog(std::string(this->get_namespace()) + std::string(this->get_name())) << "Publishing Pointcloud topic";
        }
        else
        {
          infoLog(std::string(this->get_namespace()) + std::string(this->get_name()))
            << "Stopped publishing of Pointcloud topic";
        }
      }
    }

    return result;
  };

  m_paramsCbHandle = add_on_set_parameters_callback(parameterChangeCallback);

  get_parameter("serial_number", m_serialNumber);
}

} // namespace sick

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sick::PointCloudPublisher)
