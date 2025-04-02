// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_POINTCLOUDPUBLISHER_HPP
#define SICK_PUBLISHER_SRC_POINTCLOUDPUBLISHER_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "sick_publisher/CameraIntrinsics.hpp"

namespace rclcpp
{
class NodeOptions;
}

namespace sick
{

/// @brief VisionaryPublisher class.
/// @details This class is responsible for publishing the data from the camera.

class PointCloudPublisher : public rclcpp::Node
{
public:
  /// brief Constructor.
  /// @param options Node options for rclcpp.
  PointCloudPublisher(const rclcpp::NodeOptions& options);

  /// @brief Destructor.
  ~PointCloudPublisher() = default;

  /// @brief Callback for CameraInfo topic.
  /// Reads the relevant parameters for the pointcloud creation and stores them in CameraIntrinsics
  /// @param camInfo The camera info message.
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camInfo);

  /// @brief Publishes the pointcloud data constructed from the subscribed intensity and depth images.
  /// This method is called when synchronized intensity and depth images are received. It processes the images
  /// to construct a pointcloud and publishes the resulting pointcloud data.///
  /// @param intensityImg The synchronized intensity image.
  /// @param depthImg The synchronized depth image.
  void onPublishPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr& intensityImg,
                           const sensor_msgs::msg::Image::ConstSharedPtr& depthImg);

private:
  std::string m_namespace = std::string(this->get_namespace());
  std::string m_cameraFrame;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_camInfoSub;
  message_filters::Subscriber<sensor_msgs::msg::Image> m_intensityImgSub;
  message_filters::Subscriber<sensor_msgs::msg::Image> m_depthImgSub;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> m_timeSync;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudPublisher;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> m_pointCloudMsg;

  // Callbackgroups
  rclcpp::CallbackGroup::SharedPtr m_camInfoSubCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr m_subscriptionsCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr m_pointCloudCallbackGroup;

  // Parameterhandling
  void setupParameterHandling();
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_paramsCbHandle;
  int m_serialNumber;
  bool m_publishPointcloud{true};

  // Camera specifics
  bool m_cameraIntrinsicsFilled{false};
  CameraIntrinsics m_cameraIntrinsics;
  std::vector<float> m_cachedX;
  std::vector<float> m_cachedY;
};
} // namespace sick

#endif // SICK_PUBLISHER_SRC_POINTCLOUDPUBLISHER_HPP