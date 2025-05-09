/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__ROS_COMMON_HPP_
#define QRB_ROS_AMR__ROS_COMMON_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/int16.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "qrb_ros_amr_msgs/action/ae.hpp"
#include "qrb_ros_amr_msgs/action/cmd.hpp"
#include "qrb_ros_amr_msgs/srv/api.hpp"
#include "qrb_ros_amr_msgs/srv/mapping.hpp"
#include "qrb_ros_amr_msgs/srv/sub_cmd.hpp"
#include "qrb_ros_amr_msgs/msg/amr_status.hpp"
#include "qrb_ros_amr_msgs/msg/wheel_status.hpp"
#include "qrb_ros_amr_msgs/msg/battery_info.hpp"
#include "qrb_ros_navigation_msgs/action/follow_path.hpp"
#include "qrb_ros_navigation_msgs/srv/virtual_path.hpp"
#include "qrb_ros_robot_base_msgs/srv/set_control_mode.hpp"
#include "qrb_ros_robot_base_msgs/msg/charger_cmd.hpp"
#include "qrb_ros_robot_base_msgs/msg/error.hpp"
#include "qrb_ros_slam_msgs/srv/slam_command.hpp"
#include "qrb_ros_slam_msgs/msg/command_code.hpp"

using Pose = geometry_msgs::msg::PoseStamped;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using FollowPath = nav2_msgs::action::FollowPath;
using P2P = nav2_msgs::action::NavigateToPose;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using ChangeState = lifecycle_msgs::srv::ChangeState;

using GoalHandleP2P = rclcpp_action::ClientGoalHandle<P2P>;
using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowPath>;

using AE = qrb_ros_amr_msgs::action::AE;
using Cmd = qrb_ros_amr_msgs::action::Cmd;
using SubCmd = qrb_ros_amr_msgs::srv::SubCmd;
using Mapping = qrb_ros_amr_msgs::srv::Mapping;

using GoalHandleAE = rclcpp_action::ClientGoalHandle<AE>;
using GoalHandleCmd = rclcpp_action::ServerGoalHandle<Cmd>;

using WaypointFollowPath = qrb_ros_navigation_msgs::action::FollowPath;
using VirtualPath = qrb_ros_navigation_msgs::srv::VirtualPath;

using GoalHandleWaypointFollowPath = rclcpp_action::ClientGoalHandle<WaypointFollowPath>;

using SetBaseControlMode = qrb_ros_robot_base_msgs::srv::SetControlMode;
using ChargerCmd = qrb_ros_robot_base_msgs::msg::ChargerCmd;

using SlamCommand = qrb_ros_slam_msgs::srv::SlamCommand;
using CommandCode = qrb_ros_slam_msgs::msg::CommandCode;

#define SERVICE_TIMEOUT_DURATION 5  // timeout is 5 seconds

#endif  // QRB_ROS_AMR__ROS_COMMON_HPP_