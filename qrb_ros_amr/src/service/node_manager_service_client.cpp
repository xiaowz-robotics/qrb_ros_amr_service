/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "node_manager_service_client.hpp"

using namespace std::chrono_literals;

namespace qrb_ros
{
namespace amr
{
NodeManagerServiceClient::NodeManagerServiceClient(std::shared_ptr<AMRManager> & amr_manager,
    const rclcpp::NodeOptions & options)
  : Node("node_manager", options), amr_manager_(amr_manager)
{
  client_list_.push_back(this->create_client<ChangeState>("/map_sub/change_state"));
  client_list_.push_back(this->create_client<ChangeState>("/fp_node/change_state"));
  client_list_.push_back(
      this->create_client<ChangeState>("/navigation_path_service_server/change_state"));
  client_list_.push_back(
      this->create_client<ChangeState>("/virtual_path_service_server/change_state"));
  client_list_.push_back(this->create_client<ChangeState>("/debug_sub/change_state"));

  client_list_.push_back(this->create_client<ChangeState>("/exception_sub/change_state"));
  client_list_.push_back(this->create_client<ChangeState>("/laser_sub/change_state"));
  client_list_.push_back(this->create_client<ChangeState>("/odom_sub/change_state"));
  client_list_.push_back(this->create_client<ChangeState>("/path_pub/change_state"));
  client_list_.push_back(this->create_client<ChangeState>("/twist_pub/change_state"));
  client_list_.push_back(this->create_client<ChangeState>("/tf_sub/change_state"));

  execute_result_ = false;
  current_lifecycle_state = (uint32_t)Lifecycle_State::UnConfigured;

  node_manager_callback_ = [&](bool active) { activate_node(active); };
  amr_manager_->register_node_manager_callback(node_manager_callback_);
}

NodeManagerServiceClient::~NodeManagerServiceClient() {}

void NodeManagerServiceClient::send_request(const ChangeState::Request::SharedPtr request,
    rclcpp::Client<ChangeState>::SharedPtr client)
{
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(logger_, "service not available, waiting again...");
  }
  using ServiceResponseFuture = rclcpp::Client<ChangeState>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    std::unique_lock<std::mutex> lck(mtx_);
    auto result = future.get();
    execute_result_ = result->success;
    cv_.notify_one();
  };
  auto future_result = client->async_send_request(request, response_received_callback);
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);
}

bool NodeManagerServiceClient::activate_node(bool active)
{
  RCLCPP_INFO(logger_, "active node: %d", active);
  auto request = std::make_shared<ChangeState::Request>();
  if (active) {
    if (current_lifecycle_state == (uint32_t)Lifecycle_State::UnConfigured) {
      configure_node();
    }
    activate_node();
    current_lifecycle_state = (uint32_t)Lifecycle_State::Active;
  } else {
    deactivate_node();
    current_lifecycle_state = (uint32_t)Lifecycle_State::Inactive;
  }

  return execute_result_;
}

bool NodeManagerServiceClient::configure_node()
{
  RCLCPP_INFO(logger_, "configure_node");
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

  for (rclcpp::Client<ChangeState>::SharedPtr client : client_list_) {
    send_request(request, client);
  }
  return execute_result_;
}

bool NodeManagerServiceClient::activate_node()
{
  RCLCPP_INFO(logger_, "activate_node");
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

  for (rclcpp::Client<ChangeState>::SharedPtr client : client_list_) {
    send_request(request, client);
  }
  return execute_result_;
}

bool NodeManagerServiceClient::deactivate_node()
{
  RCLCPP_INFO(logger_, "deactivate_node");
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

  for (rclcpp::Client<ChangeState>::SharedPtr client : client_list_) {
    send_request(request, client);
  }
  return execute_result_;
}
}  // namespace amr
}  // namespace qrb_ros
