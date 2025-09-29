/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_AMR__AMR_MODE_SERVICE_CLIENT_HPP_
#define QRB_ROS_AMR__AMR_MODE_SERVICE_CLIENT_HPP_

#include "amr_manager.hpp"
#include "ros_common.hpp"

using namespace qrb::amr_manager;

namespace qrb_ros
{
namespace amr
{
/**
 * @enum amr_controller::AMRControlMode
 * @desc Enum class representing mode of amr control.
 */
enum class AMRControlMode
{
  Remote_Controller = 1,  // Control AMR moving by remote controller
  Navigation = 2,         // Control AMR moving by navigator
};

class AMRModeServiceClient : public rclcpp::Node
{
private:
  rclcpp::Client<SetBaseControlMode>::SharedPtr client_;
  std::shared_ptr<AMRManager> amr_manager_;
  std::mutex mtx_;
  std::condition_variable cv_;
  bool execute_result_;
  qrb::amr_manager::change_mode_func_t change_mode_callback_;
  rclcpp::Logger logger_{ rclcpp::get_logger("amr_mode_service_client") };

  void send_request(const SetBaseControlMode::Request::SharedPtr request);

public:
  bool change_mode(uint8_t mode);
  bool reset_odometer();
  explicit AMRModeServiceClient(std::shared_ptr<AMRManager> & amr_manager,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AMRModeServiceClient();
};
}  // namespace amr
}  // namespace qrb_ros
#endif  // QRB_ROS_AMR__AMR_MODE_SERVICE_CLIENT_HPP_
