/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__SUB_CMD_SERVICE_SERVER_HPP_
#define QRB_ROS_AMR__SUB_CMD_SERVICE_SERVER_HPP_

#include "amr_manager.hpp"
#include "ros_common.hpp"

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{

class SubCmdServer : public rclcpp::Node
{
private:
  rclcpp::Service<qrb_ros_amr_msgs::srv::SubCmd>::SharedPtr srv_ptr_;
  std::shared_ptr<AMRManager> amr_manager_;

  void init_service();
  void handle_sub_cmd(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_amr_msgs::srv::SubCmd::Request> request,
      std::shared_ptr<qrb_ros_amr_msgs::srv::SubCmd::Response> response);

public:
  SubCmdServer(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SubCmdServer();
};
}  // namespace amr
}  // namespace qrb_ros

#endif  // QRB_ROS_AMR__SUB_CMD_SERVICE_SERVER_HPP_