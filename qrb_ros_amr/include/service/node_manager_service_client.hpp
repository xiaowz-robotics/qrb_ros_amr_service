/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__NODE_MANAGER_SERVICE_CLIENT_HPP_
#define QRB_ROS_AMR__NODE_MANAGER_SERVICE_CLIENT_HPP_

#include "amr_manager.hpp"
#include "ros_common.hpp"

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{
class NodeManagerServiceClient : public rclcpp::Node
{
private:
  rclcpp::Client<ChangeState>::SharedPtr client_;
  std::vector<rclcpp::Client<ChangeState>::SharedPtr> client_list_;
  std::shared_ptr<AMRManager> amr_manager_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool execute_result_;
  uint32_t current_lifecycle_state;
  qrb::amr_manager::node_manager_func_t node_manager_callback_;
  rclcpp::Logger logger_{ rclcpp::get_logger("node_manager_service_client") };

  void send_request(const ChangeState::Request::SharedPtr request,
      rclcpp::Client<ChangeState>::SharedPtr client);

  bool activate_node(bool active);

  bool configure_node();

  bool activate_node();

  bool deactivate_node();

  bool cleanup_node();

public:
  explicit NodeManagerServiceClient(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NodeManagerServiceClient();
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__NODE_MANAGER_SERVICE_CLIENT_HPP_
