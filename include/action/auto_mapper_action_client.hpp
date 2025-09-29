/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__AUTO_MAPPER_ACTION_CLIENT_HPP_
#define QRB_ROS_AMR__AUTO_MAPPER_ACTION_CLIENT_HPP_

#include "amr_manager.hpp"
#include "ros_common.hpp"
#include "cmd_action_server.hpp"

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{

class AutoMapperActionClient : public rclcpp::Node
{
private:
  rclcpp_action::Client<AE>::SharedPtr client_ptr_;
  std::shared_ptr<AMRManager> amr_manager_;

  void create_client();
  void send_goal();
  void goal_response_callback(GoalHandleAE::SharedPtr goal_handle);
  void feedback_callback(GoalHandleAE::SharedPtr,
      const std::shared_ptr<const AE::Feedback> feedback);
  void result_callback(const GoalHandleAE::WrappedResult & result);

public:
  AutoMapperActionClient(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AutoMapperActionClient();
  void start_ae();
  void cancel_goal();
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__AUTO_MAPPER_ACTION_CLIENT_HPP_