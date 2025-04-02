// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "PointCloudPublisher.hpp"
#include "SICKPublisher.hpp"
#include "base/CmdLineFlags.hpp"
#include "base/Logging.hpp"
#include "sick_publisher/CameraIntrinsics.hpp"
#include "utils/ParamsFileUtils.hpp"

namespace sick
{

DEFINE_bool(publish_pointcloud, true, "Toggle pointcloud publishing");

namespace internal
{
void printUsageAndExit()
{
  fprintf(stdout,
          "manual_composition"
          "  [--publish_pointcloud={true|false}]\n");
  std::exit(0);
}

void parseCmdLineFlags(int argc, char* argv[])
{
  using namespace sick;
  for (int idx = 1; idx < argc; ++idx)
  {
    if (parseBoolFlag(argv[idx], "publish_pointcloud", &FLAGS_publish_pointcloud))
    {
      for (int jdx = idx; jdx != argc - 1; ++jdx)
      {
        argv[jdx] = argv[jdx + 1];
      }
      --(argc);
      --idx;
    }
    else if (isFlag(argv[idx], "help"))
    {
      printUsageAndExit();
    }
  }
}
} // namespace internal
} // namespace sick

int main(int argc, char* argv[])
{
  using namespace sick;
  // Flag to flush stdout buffer after every output.
  std::cout.setf(std::ios_base::unitbuf);

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  internal::parseCmdLineFlags(argc, argv);

  // Parse serial numbers from YAML file
  namespace fs = std::filesystem;
  const std::string shareDir = ament_index_cpp::get_package_share_directory("sick_visionary_gev_ros2");
  const std::string configFp = fs::path(shareDir + "/config/params.yaml").make_preferred().string();
  auto serialNumbersMap = parseSerialNumbers(configFp);

  std::vector<std::shared_ptr<VisionaryPublisher>> visionaryPubNodes;
  std::vector<std::shared_ptr<PointCloudPublisher>> pointcloudPubNodes;

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  rclcpp::executors::MultiThreadedExecutor exec;

  std::mutex mtx;
  for (const auto& [modelName, serialNumbers] : serialNumbersMap)
  {
    for (const auto& serial : serialNumbers)
    {
      if (!serial.empty())
      {
        std::lock_guard<std::mutex> lock(mtx);
        infoLog("ManualComposition.cpp") << "Initialize VisionaryPublisher-Node: " << serial;

        // VisionaryPublisher Node
        rclcpp::NodeOptions visionaryPubOptions;
        visionaryPubOptions.parameter_overrides({rclcpp::Parameter("serial_number", std::atoi(serial.c_str()))});
        std::string visionaryNamespace = "__ns:=/" + modelName;
        std::string visionaryPubName = "__node:=cam_" + serial;
        visionaryPubOptions.arguments({"--ros-args", "-r", visionaryNamespace, "-r", visionaryPubName});

        // Add nodes to the executor which provide work for the executor during its "spin" function.
        auto visionaryPubNode = std::make_shared<VisionaryPublisher>(visionaryPubOptions);
        exec.add_node(visionaryPubNode);
        visionaryPubNodes.push_back(visionaryPubNode);

        if (FLAGS_publish_pointcloud)
        {
          infoLog("ManualComposition.cpp") << "Initialize PointCloudPublisher-Node: " << serial;
          rclcpp::NodeOptions pointcloudPubOptions;
          pointcloudPubOptions.parameter_overrides({rclcpp::Parameter("serial_number", std::atoi(serial.c_str()))});
          std::string pointcloudPubNamespace = "__ns:=/" + modelName;
          std::string pointcloudPubName = "__node:=pointcloud_publisher_" + serial;
          pointcloudPubOptions.arguments({"--ros-args", "-r", pointcloudPubName, "-r", pointcloudPubNamespace});

          auto pointcloudPubNode = std::make_shared<PointCloudPublisher>(pointcloudPubOptions);
          exec.add_node(pointcloudPubNode);
          pointcloudPubNodes.push_back(pointcloudPubNode);
        }
      }
    }
  }

  // Spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
