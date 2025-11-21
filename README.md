<div align="center">
  <h1>QRB ROS AMR Service</h1>
  <p align="center">
   <img src="https://s7d1.scene7.com/is/image/dmqualcommprod/rb3gen2-dev-kits-hero-7" alt="Qualcomm QRB ROS" title="Qualcomm QRB ROS" />
      
  </p>
  <p>ROS2 Package for Autonomous Mobile Robot (AMR) Service on Qualcomm Robotics Platform</p>
  
  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
  
</div>

---

## 👋 Overview

[QRB ROS AMR Service](https://github.com/qualcomm-qrb-ros/qrb_ros_amr_service) is a ROS2 package suite designed to manage AMR behaviors on Qualcomm robotics platform. Key features include:

- P2P navigation requests.
- Path following.
- Mapping services.
- Automatic return to charging station when battery is low.

<div align="center">
  <img src="/docs/assets/architecture.png" alt="architecture">
</div>

<br>

[`qrb_ros_amr`](https://github.com/qualcomm-qrb-ros/qrb_ros_amr_service/tree/main/qrb_ros_amr): ROS2 package implementing action/service clients & servers, publishers and subscribers for navigation, mapping and AMR status feedback.

[`qrb_amr_manager`](https://github.com/qualcomm-qrb-ros/qrb_ros_amr_service/tree/main/qrb_amr_manager): C++ library providing APIs for AMR state management and charging logic.
- Low Power Manager: Returns to charging station when battery < 2.2V.
- Manager: Library interface.
- State Machine: Manages AMR state and actions.

[`Follow Path Service & Follow Path Manager](https://github.com/qualcomm-qrb-ros/qrb_ros_follow_path_service): ROS2 package for path following navigation. 

[`Robot Base & Robot Base Controller](https://github.com/qualcomm-qrb-ros/qrb_ros_robot_base): ROS2 package for AMR base control.

`2D LiDAR SLAM ROS & 2D LiDAR SLAM`: Provides mapping and localization.

`Nav2`: Provides P2P navigation.

## 🔎 Table of contents
  * [APIs](#-apis)
     * [`qrb_ros_amr` APIs](#-qrb_ros_amr_service-apis)
     * [`qrb_amr_manager` APIs](#-qrb_amr_manager-apis)
  * [Supported targets](#-supported-targets)
  * [Installation](#-installation)
  * [Build from source](#-build-from-source)
  * [Usage](#-usage)
     * [Starting the amr service node](#start-the-amr-service-node)
  * [Contributing](#-contributing)
  * [Contributors](#%EF%B8%8F-contributors)
  * [FAQs](#-faqs)
  * [License](#-license)

## ⚓ APIs

### 🔹 `qrb_ros_amr_service` APIs

#### ROS interfaces

<table>
  <tr>
    <th>Interface</th>
    <th>Name</th>
    <th>Type</th>
    <td>Description</td>
  </tr>
  <tr>
    <td>Publisher</td>
    <td>amr_status</td>
    <td>qrb_ros_amr_msgs::msg::AMRStatus</td>
    <td>Publishes AMR status</td>
  </tr>
  <tr>
    <td>Subscriber</td>
    <td>test</td>
    <td>std_msgs::msg::Int16</td>
    <td>Receives debugging information</td>
  </tr>
  <tr>
    <td>Subscriber</td>
    <td>debug_exception</td>
    <td>std_msgs::msg::Int16</td>
    <td>Receives simulated exception info</td>
  </tr>
  <tr>
    <td>Service Server</td>
    <td>sub_cmd</td>
    <td>qrb_ros_amr_msgs::srv::SubCmd</td>
    <td>Handles sub-command requests</td>
  </tr>
  <tr>
    <td>Service Server</td>
    <td>amr_mapping</td>
    <td>qrb_ros_amr_msgs::srv::Mapping</td>
    <td>Handles mapping requests</td>
  </tr>
  <tr>
    <td>Service Server</td>
    <td>api</td>
    <td>qrb_ros_amr_msgs::srv::API</td>
    <td>Handles API requests</td>
  </tr>
  <tr>
    <td>Action Server</td>
    <td>cmd</td>
    <td>qrb_ros_amr_msgs::action::Cmd</td>
    <td>Handles navigation commands</td>
  </tr>
</table>

#### ROS message parameters

##### amr_status
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>status_change_id</td>
    <td>uint8</td>
    <td>AMR changed status ID</td>
    <td>-</td>
  </tr>
  <tr>
    <td>amr_exception</td>
    <td>bool</td>
    <td>AMR exception information</td>
    <td>-</td>
  </tr>
  <tr>
    <td>error_code</td>
    <td>uint8</td>
    <td>Error code when AMR is in exception</td>
    <td>-</td>
  </tr>
  <tr>
    <td>current_pose</td>
    <td>geometry_msgs/PoseStamped</td>
    <td>Current position of AMR</td>
    <td>-</td>
  </tr>
  <tr>
    <td>current_state</td>
    <td>uint8</td>
    <td>Current state of AMR</td>
    <td>-</td>
  </tr>
  <tr>
    <td>battery_vol</td>
    <td>float64</td>
    <td>Current battery voltage</td>
    <td>-</td>
  </tr>
  <tr>
    <td>vel</td>
    <td>geometry_msgs/Vector3</td>
    <td>Current velocity of AMR</td>
    <td>-</td>
  </tr>
  <tr>
    <td>mileage</td>
    <td>float64</td>
    <td>Reserved</td>
    <td>-</td>
  </tr>
  <tr>
    <td>header</td>
    <td>std_msgs/Header</td>
    <td>Message header</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can interact with these interfaces for AMR monitoring.

##### test
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>data</td>
    <td>int16</td>
    <td>Debugging ID</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send debugging information to simulate AMR events in developer mode.

##### debug_exception
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>data</td>
    <td>int16</td>
    <td>Debugging exception ID</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send debugging exceptions to simulate AMR exception events in developer mode.

##### sub_cmd
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>subcommand</td>
    <td>uint8</td>
    <td>Sub-command</td>
    <td>-</td>
  </tr>
  <tr>
    <td>result</td>
    <td>bool</td>
    <td>Result of sub-command</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send sub-commands to cancel, pause, or resume navigation.

##### amr_mapping
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>cmd</td>
    <td>uint8</td>
    <td>Mapping command</td>
    <td>-</td>
  </tr>
  <tr>
    <td>result</td>
    <td>bool</td>
    <td>Result of command</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send mapping commands to load map, start mapping, or stop mapping.

##### api
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>api_id</td>
    <td>uint8</td>
    <td>API ID</td>
    <td>-</td>
  </tr>
  <tr>
    <td>developer_mode</td>
    <td>bool</td>
    <td>Enable/disable developer mode</td>
    <td>-</td>
  </tr>
  <tr>
    <td>result</td>
    <td>bool</td>
    <td>Result of API handling</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send API requests to initialize AMR, release AMR, or control developer mode.

##### cmd
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>command</td>
    <td>uint8</td>
    <td>Command ID (navigation, charging, etc.)</td>
    <td>-</td>
  </tr>
  <tr>
    <td>goal</td>
    <td>geometry_msgs/PoseStamped</td>
    <td>Goal for point-to-point navigation</td>
    <td>-</td>
  </tr>
  <tr>
    <td>path</td>
    <td>nav_msgs/Path</td>
    <td>Reserved</td>
    <td>-</td>
  </tr>
  <tr>
    <td>goal_id</td>
    <td>uint32</td>
    <td>Target waypoint ID for follow path</td>
    <td>-</td>
  </tr>
  <tr>
    <td>passing_waypoint_ids</td>
    <td>uint32[]</td>
    <td>Waypoints to pass through on follow path</td>
    <td>-</td>
  </tr>
  <tr>
    <td>result</td>
    <td>bool</td>
    <td>Result of command</td>
    <td>-</td>
  </tr>
  <tr>
    <td>current_pose</td>
    <td>geometry_msgs/PoseStamped</td>
    <td>Current position of AMR</td>
    <td>-</td>
  </tr>
  <tr>
    <td>passing_waypoint_id</td>
    <td>uint32</td>
    <td>Passing waypoint ID during follow path</td>
    <td>-</td>
  </tr>
  <tr>
    <td>distance_to_goal</td>
    <td>float32</td>
    <td>Distance from current position to goal</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send command requests to start point-to-point navigation, follow path, or return to charging station.
  
### 🔹 `qrb_amr_manager` APIs

<table>
  <tr>
    <th>Function</th>
    <th>Parameters</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>void init_amr()</td>
    <td>-</td>
    <td>Initialize AMR service.</td>
  </tr>
  <tr>
    <td>void release_amr()</td>
    <td>-</td>
    <td>Release AMR service.</td>
  </tr>
  <tr>
    <td>bool check_potential_state(int cmd)</td>
    <td>cmd: Navigation command (P2PNav, FollowPath, Return Charging)</td>
    <td>Check if AMR state meets navigation requirements.</td>
  </tr>
  <tr>
    <td>bool check_potential_subcmd_state(uint8_t subcmd)</td>
    <td>subcmd: Sub-command (Cancel, Pause, Resume)</td>
    <td>Check if AMR state meets sub-command requirements.</td>
  </tr>
  <tr>
    <td>void process_cmd(int cmd, void * buffer, size_t len)</td>
    <td>cmd: Navigation command, buffer: Goal, len: Length of goal</td>
    <td>Start P2P or charging station navigation.</td>
  </tr>
  <tr>
    <td>void process_cmd(int cmd, uint32_t goal_id, std::vector<uint32_t> & ids)</td>
    <td>cmd: Follow path command, goal_id: Target waypoint, ids: Passing waypoints</td>
    <td>Start follow path navigation.</td>
  </tr>
  <tr>
    <td>bool start_mapping()</td>
    <td>-</td>
    <td>Start mapping.</td>
  </tr>
  <tr>
    <td>bool stop_mapping()</td>
    <td>-</td>
    <td>Stop mapping.</td>
  </tr>
  <tr>
    <td>bool load_map()</td>
    <td>-</td>
    <td>Load map.</td>
  </tr>
  <tr>
    <td>void process_sub_cmd(int msg)</td>
    <td>msg: Sub-command</td>
    <td>Cancel, pause, or resume navigation (including follow path and charging).</td>
  </tr>
  <tr>
    <td>void notify_battery_changed(float battery_vol)</td>
    <td>battery_vol: Battery voltage</td>
    <td>Update battery voltage.</td>
  </tr>
  <tr>
    <td>void notify_charging_state_changed(uint8_t state)</td>
    <td>state: Charging state</td>
    <td>Update charging state.</td>
  </tr>
  <tr>
    <td>void register_start_charging_callback(start_charging_func_t cb)</td>
    <td>cb: Start charging callback</td>
    <td>Register callback for starting charging.</td>
  </tr>
  <tr>
    <td>void register_publish_twist_callback(publish_twist_func_t cb)</td>
    <td>cb: Move AMR callback</td>
    <td>Register callback for moving AMR.</td>
  </tr>
  <tr>
    <td>void register_start_p2p_nav_callback(start_p2p_func_t cb)</td>
    <td>cb: P2P navigation completion callback</td>
    <td>Register callback for P2P navigation completion.</td>
  </tr>
  <tr>
    <td>void register_start_waypoint_follow_path_callback(start_waypoint_follow_path_func_t cb)</td>
    <td>cb: Follow path completion callback</td>
    <td>Register callback for follow path completion.</td>
  </tr>
  <tr>
    <td>register_navigate_to_charging_callback(navigate_to_charging_func_t cb)</td>
    <td>cb: Charging navigation completion callback</td>
    <td>Register callback for charging navigation completion.</td>
  </tr>
  <tr>
    <td>void register_sub_cmd_callback(sub_cmd_func_t cb)</td>
    <td>cb: Sub-command callback</td>
    <td>Register callback for sub-command.</td>
  </tr>
  <tr>
    <td>void register_change_mode_callback(change_mode_func_t cb)</td>
    <td>cb: Control mode callback</td>
    <td>Register callback to set control mode (application, charging, remote controller).</td>
  </tr>
  <tr>
    <td>void register_slam_command_callback(slam_command_func_t cb)</td>
    <td>cb: SLAM command callback</td>
    <td>Register callback for SLAM commands (start/stop mapping, save/load map, localization, relocalization).</td>
  </tr>
</table>

> [!Note]
> These APIs enable AMR service integration for non-ROS applications. For ROS packages, please use the qrb_ros_amr_service APIs..

## 🎯 Supported targets

<table >
  <tr>
    <th>Development Hardware</th>
    <td>Qualcomm Dragonwing™ RB3 Gen2</td>
    <td>Qualcomm Dragonwing™ IQ-9075 EVK</td>
  </tr>
  <tr>
    <th>Hardware Overview</th>
    <th><a href="https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit"><img src="https://s7d1.scene7.com/is/image/dmqualcommprod/rb3-gen2-carousel?fmt=webp-alpha&qlt=85" width="180"/></a></th>
    <th><a href="https://www.qualcomm.com/products/internet-of-things/industrial-processors/iq9-series/iq-9075"><img src="https://s7d1.scene7.com/is/image/dmqualcommprod/dragonwing-IQ-9075-EVK?$QC_Responsive$&fmt=png-alpha" width="160"></a></th>
  </tr>
</table>

---

## ✨ Installation

> [!IMPORTANT]
> **PREREQUISITES**: The following steps need to be run on **Qualcomm Ubuntu** and **ROS Jazzy**.<br>
> Reference [Install Ubuntu on Qualcomm IoT Platforms](https://ubuntu.com/download/qualcomm-iot) and [Install ROS Jazzy](https://docs.ros.org/en/jazzy/index.html) to setup environment. <br>
> For Qualcomm Linux, please check out the [Qualcomm Intelligent Robotics Product SDK](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm%20Intelligent%20Robotics%20Product%20(QIRP)%20SDK) documents.

## 👨‍💻 Build from source

Install dependencies:

```bash
sudo apt update
sudo apt install colcon build-essential g++
sudo apt install ros-jazzy-nav2-msgs
sudo apt install ros-jazzy-nav-2d-msgs
```

Clone:
```bash
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_amr_service.git
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_interfaces.git
```
Keep only these 4 packages(qrb_ros_amr_msgs/qrb_ros_navigation_msgs/qrb_ros_slam_msgs/qrb_ros_robot_base_msgs) in qrb_ros_interfaces and delete the others.

Build:
```bash
source /opt/ros/jazzy/setup.bash
colcon build
```
If an exception occurs during build, please use the following command to build.
```bash
source /opt/ros/jazzy/setup.bash
colcon build --parallel-workers 1
```

## 👨‍💻 Install from Qualcomm IOT PPA
Developers can also choose to install directly instead of downloading and compiling the source code.

Add Qualcomm IOT PPA for Ubuntu:

```bash
sudo add-apt-repository ppa:ubuntu-qcom-iot/qcom-noble-ppa
sudo add-apt-repository ppa:ubuntu-qcom-iot/qirp
sudo apt update
```

Install Debian package:

```bash
sudo apt install ros-jazzy-nav2-msgs
sudo apt install ros-jazzy-nav-2d-msgs
sudo apt install ros-jazzy-qrb-ros-amr-msgs
sudo apt install ros-jazzy-qrb-ros-navigation-msgs
sudo apt install ros-jazzy-qrb-amr-manager
sudo apt install ros-jazzy-qrb-ros-amr
```

## 🚀 Usage

### Start the amr service node

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch qrb_ros_amr qrb_ros_amr_bringup.launch.py
```

Sample output:

```bash
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2025-09-17-10-58-59-978240-ubuntu-3342
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [qrb_ros_amr-1]: process started with pid [3345]
[qrb_ros_amr-1] [amr_manager]: AMRManager
[qrb_ros_amr-1] [amr_manager]: init
[qrb_ros_amr-1] [INFO] [1758106740.351087121] [mapping_service_server]: MappingServiceServer
[qrb_ros_amr-1] [amr_state_machine]: register_start_p2p_nav_callback
[qrb_ros_amr-1] [amr_state_machine]: register_start_follow_path_callback
[qrb_ros_amr-1] [amr_state_machine]: register_start_waypoint_follow_path_callback
[qrb_ros_amr-1] [amr_state_machine]: register_sub_cmd_callback
[qrb_ros_amr-1] [amr_state_machine]: register_navigate_to_charging_callback
[qrb_ros_amr-1] [INFO] [1758106740.403569396] [cartographer_service_client]: Creating
[qrb_ros_amr-1] [INFO] [1758106740.403678772] [cartographer_service_client]: init_client
[qrb_ros_amr-1] [amr_state_machine]: register_slam_command_callback
[qrb_ros_amr-1] [low_power_manager]: register_change_mode_callback
[qrb_ros_amr-1] [INFO] [1758106740.417741601] [amr_status_transporter]: AMRStatusTransporter
[qrb_ros_amr-1] [amr_state_machine]: register_start_charging_callback
[qrb_ros_amr-1] [amr_state_machine]: register_notify_exception_callback
[qrb_ros_amr-1] [amr_state_machine]: register_send_amr_state_changed_callback
[qrb_ros_amr-1] [amr_state_machine]: register_publish_twist_callback
[qrb_ros_amr-1] [INFO] [1758106740.434623101] [developer_mode_subscriber]: DeveloperModeSubscriber
[qrb_ros_amr-1] [amr_state_machine]: register_node_manager_callback
[qrb_ros_amr-1] [INFO] [1758106740.456975064] [amr_controller]: Init amr node
...
```

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).<br>
Feel free to create an issue for bug report, feature requests or any discussion💡.

## ❤️ Contributors

Thanks to all our contributors who have helped make this project better!

<table>
  <tr>
    <td align="center"><a href="https://github.com/quic-zhanlin"><img src="https://avatars.githubusercontent.com/u/88314584?v=4" width="100" height="100" alt="quic-zhanlin"/><br /><sub><b>quic-zhanlin</b></sub></a></td>
    <td align="center"><a href="https://github.com/xiaowz-robotics"><img src="https://avatars.githubusercontent.com/u/154509668?v=4" width="100" height="100" alt="xiaowz-robotics"/><br /><sub><b>xiaowz-robotics</b></sub></a></td>
  </tr>
</table>

## ❔ FAQs

<details>
<summary>Does it support other AMR?</summary><br>
No, it only supports Qualcomm AMR.
</details>

## 📜 License

Project is licensed under the [BSD-3-Clause](https://spdx.org/licenses/BSD-3-Clause.html) License. See [LICENSE](./LICENSE) for the full license text.
