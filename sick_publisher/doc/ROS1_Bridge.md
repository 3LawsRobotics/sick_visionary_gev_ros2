# Enabling ROS1 Bridge for SICKPublisher
This documentation provides detailed steps for setting up the ROS1 Bridge to enable communication
between the `SICKPublisher` ROS2 node and ROS1 users. The ROS1 Bridge facilitates message
translation, allowing ROS1 applications to use topics published by ROS2 nodes.

## Prerequisites
Ensure that the following software versions are installed on your system:

- **Ubuntu 20.04** (the last Ubuntu version supported for ROS1 Noetic)
- **ROS1 Noetic** (for ROS1 support)
- **ROS2 Humble or Iron** (for ROS2 support)

Note that ROS1 Noetic is the latest and final distribution for ROS1 which is supported. Older
versions of ROS1 are not supported anymore.

Both ROS1 and ROS2 distributions must be installed on the same machine for the bridge to work.

## Step 1: Install ROS1 Noetic
If ROS1 Noetic is not already installed, follow these steps:

1. Update the package index:
   ```bash
   sudo apt update
   ```

2. Install ROS1 Noetic Desktop Full:
   ```bash
   sudo apt install ros-noetic-desktop-full
   ```

3. Source the ROS1 setup file:
   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Step 2: Install ROS2 (Humble or Iron)
Follow the official installation instructions for your ROS2 distribution. Once installed, source the
ROS2 setup file:

```bash
source /opt/ros/<ros2_distro>/setup.bash
```
Replace `<ros2_distro>` with either `humble` or `iron`, depending on your installation.

## Step 3: Install the ROS1 Bridge
To set up the ROS1 Bridge, install the `ros1_bridge` package for your ROS2 distribution:

```bash
sudo apt install ros-<ros2_distro>-ros1-bridge
```
Replace `<ros2_distro>` with `humble` or `iron`.

## Step 4: Source Both ROS Environments
The bridge requires both ROS1 and ROS2 environments to be active. You can do this by running:

```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/<ros2_distro>/setup.bash
```

## Step 5: Launch the ROS1 Bridge

### Option 1: Dynamic Bridge (Recommended)
The dynamic bridge automatically handles message translation for any matching topic types it
detects. To launch the dynamic bridge:

```bash
ros2 run ros1_bridge dynamic_bridge
```

### Option 2: Static Bridge (If Needed)
If you need to specify exact topics and message types for bridging, use the static bridge
(less common). Refer to the [ROS1 Bridge documentation](https://index.ros.org/p/ros1_bridge/) for
more details.

## Step 6: Test the Bridge
1. **Run the SICKPublisher Node**: Launch your `SICKPublisher` ROS2 node as usual.
2. **Verify Topics in ROS1**: Open a new terminal, source the ROS1 environment, and run:
   ```bash
   rostopic list
   ```
   You should see the bridged topics from the `SICKPublisher` node.
3. **Verify Topics in ROS2**: In a ROS2 environment, run:
   ```bash
   ros2 topic list
   ```
   The topics should be visible in both ROS1 and ROS2 environments.

## Notes on Custom Message Types
If your `SICKPublisher` node uses custom message types, ensure that these message definitions are
available in both ROS1 and ROS2 environments:

1. Compile and source the custom messages in a ROS1 workspace.
2. Ensure the same messages are available and sourced in the ROS2 workspace.

## Troubleshooting
- **Topics Not Appearing**: Make sure both ROS1 and ROS2 environments are sourced correctly and that
- the `ros1_bridge` is running.
- **Message Translation Issues**: Check that all message types used by `SICKPublisher` are supported
- by the bridge. Refer to the [ROS1 Bridge documentation](https://index.ros.org/p/ros1_bridge/) for
- details on supported message types.

## Additional Resources
- [ROS1 Bridge Documentation](https://index.ros.org/p/ros1_bridge/)
- [ROS1 Noetic Installation Guide](http://wiki.ros.org/noetic/Installation)
- [ROS2 Installation Guide](https://docs.ros.org/en/rolling/Installation.html)

By following these steps, ROS1 users should be able to subscribe to topics published by the
`SICKPublisher` node via the ROS1 Bridge. If you encounter any issues, feel free to consult the
troubleshooting section or the official ROS documentation.
