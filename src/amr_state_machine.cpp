/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amr_state_machine.hpp"

namespace qrb
{
namespace amr_manager
{

AMRStateMachine::AMRStateMachine()
{
  running_ = true;
  current_state_ = IN_ACTIVE;
  thread_handle_msg_ =
      std::make_shared<std::thread>(std::mem_fn(&AMRStateMachine::handle_msg), this);
  send_mapper_cmd_ = false;
  send_navigator_cmd_ = false;
  low_power_ = false;
}

AMRStateMachine::~AMRStateMachine()
{
  running_ = false;
  queue_.notify();
  if (thread_handle_msg_->joinable()) {
    thread_handle_msg_->join();
  }
}

void AMRStateMachine::register_start_p2p_nav_callback(start_p2p_func_t cb)
{
  printf("[%s]: register_start_p2p_nav_callback\n", logger_);
  start_p2p_cb_ = cb;
}

void AMRStateMachine::register_start_follow_path_callback(start_follow_path_func_t cb)
{
  printf("[%s]: register_start_follow_path_callback\n", logger_);
  start_follow_path_cb_ = cb;
}

void AMRStateMachine::register_start_waypoint_follow_path_callback(
    start_waypoint_follow_path_func_t cb)
{
  printf("[%s]: register_start_waypoint_follow_path_callback\n", logger_);
  start_waypoint_follow_path_cb_ = cb;
}

void AMRStateMachine::register_sub_cmd_callback(sub_cmd_func_t cb)
{
  printf("[%s]: register_sub_cmd_callback\n", logger_);
  sub_cmd_cb_ = cb;
}

void AMRStateMachine::register_start_charging_callback(start_charging_func_t cb)
{
  printf("[%s]: register_start_charging_callback\n", logger_);
  start_charging_cb_ = cb;
}

void AMRStateMachine::register_notify_exception_callback(notify_exception_func_t cb)
{
  printf("[%s]: register_notify_exception_callback\n", logger_);
  notify_exception_cb_ = cb;
}

void AMRStateMachine::register_send_amr_state_changed_callback(send_amr_state_changed_func_t cb)
{
  printf("[%s]: register_send_amr_state_changed_callback\n", logger_);
  send_amr_state_changed_cb_ = cb;
}

void AMRStateMachine::register_navigate_to_charging_callback(navigate_to_charging_func_t cb)
{
  printf("[%s]: register_navigate_to_charging_callback\n", logger_);
  navigate_to_charging_cb_ = cb;
}

void AMRStateMachine::register_slam_command_callback(slam_command_func_t cb)
{
  printf("[%s]: register_slam_command_callback\n", logger_);
  slam_command_cb_ = cb;
}

void AMRStateMachine::register_publish_twist_callback(publish_twist_func_t cb)
{
  printf("[%s]: register_publish_twist_callback\n", logger_);
  publish_twist_cb_ = cb;
}

void AMRStateMachine::register_node_manager_callback(node_manager_func_t cb)
{
  printf("[%s]: register_node_manager_callback\n", logger_);
  node_manager_cb_ = cb;
}

void AMRStateMachine::init_amr()
{
  Message msg;
  msg.type = Message::INIT_AMR;
  queue_.push(msg);
}

void AMRStateMachine::release_amr()
{
  Message msg;
  msg.type = Message::RELEASE_AMR;
  queue_.push(msg);
}

bool AMRStateMachine::load_map()
{
  bool result;
  slam_command_cb_((uint8_t)Slam_Command::LoadMap, result);
  if (!result) {
    printf("[%s]: Load map failed\n", logger_);
    return false;
  }

  send_me_finish_message();
  return true;
}

bool AMRStateMachine::start_mapping()
{
  bool result;
  if (current_state_ != AMRStateMachine::IDLE) {
    printf("[%s]: Start mapping failed because current state=%s\n", logger_,
        get_current_state().c_str());
    return false;
  }

  slam_command_cb_((uint8_t)Slam_Command::StartMapping, result);
  if (!result) {
    printf("[%s]: Start mapping failed\n", logger_);
    return false;
  }

  send_me_message();
  return true;
}

bool AMRStateMachine::stop_mapping()
{
  bool result;
  slam_command_cb_((uint8_t)Slam_Command::StopMapping, result);
  if (!result) {
    printf("[%s]: Stop mapping failed\n", logger_);
    return false;
  }

  send_me_completed_message();
  return true;
}

void AMRStateMachine::process_cmd(int cmd, void * buffer, size_t len)
{
  Message msg;
  msg.type = cmd;
  msg.param = buffer;
  msg.len = len;
  queue_.push(msg);
}

void AMRStateMachine::process_cmd(int cmd, uint32_t goal_id, vector<uint32_t> & ids)
{
  Message msg;
  msg.type = cmd;
  msg.goal_id = goal_id;
  msg.ids.assign(ids.begin(), ids.end());
  queue_.push(msg);
}

void AMRStateMachine::process_sub_cmd(int message)
{
  Message msg;
  msg.type = message;
  queue_.push(msg);
}

void AMRStateMachine::process_event(int event, uint8_t error_code)
{
  Message msg;
  msg.type = event;
  msg.error_code = error_code;
  queue_.push(msg);
}

void AMRStateMachine::process_event(int event)
{
  Message msg;
  msg.type = event;
  queue_.push(msg);
}

void AMRStateMachine::handle_msg()
{
  Message msg;
  bool execute = true;
  while (running_) {
    queue_.wait(msg);
    execute = handle_message(msg);
    if (!execute) {
      handle_failed(msg);
      continue;
    }
  }
}

bool AMRStateMachine::handle_message(const Message & msg)
{
  printf("[%s]: Receive message: %s\n", logger_, Message::msg_to_string(msg.type).c_str());

  int last_state = current_state_;
  switch (msg.type) {
    case Message::INIT_AMR:
      if (current_state_ == AMRStateMachine::IN_ACTIVE) {
        activate_node();
        if (!has_map_) {
          update_state(last_state, AMRStateMachine::IDLE);
        } else {
          update_state(last_state, AMRStateMachine::READY);
        }
      }
      break;
    case Message::RELEASE_AMR:
      if ((current_state_ == AMRStateMachine::IDLE) || (current_state_ == AMRStateMachine::READY)) {
        deactivate_node();
        update_state(last_state, AMRStateMachine::IN_ACTIVE);
      }
      break;
    case Message::ME_COMPLETED:
      if (current_state_ != AMRStateMachine::ON_ME) {
        return false;
      }
      enter_me_done_state();
      update_state(last_state, AMRStateMachine::ME_DONE);
      break;
    case Message::ME:
      if (current_state_ != AMRStateMachine::IDLE && current_state_ != AMRStateMachine::READY) {
        return false;
      }
      enter_on_me_state();
      update_state(last_state, AMRStateMachine::ON_ME);
      break;
    case Message::AE:
      if (current_state_ != AMRStateMachine::IDLE && current_state_ != AMRStateMachine::READY) {
        return false;
      }
      enter_on_ae_state();
      update_state(last_state, AMRStateMachine::ON_AE);
      break;
    case Message::P2PNAV:
      if (current_state_ != AMRStateMachine::READY &&
          current_state_ != AMRStateMachine::FOLLOW_PATH_WAIT &&
          current_state_ != AMRStateMachine::P2PNAV_WAIT) {
        return false;
      }
      if (current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT) {
        printf("[%s]: Cancel last follow path\n", logger_);
        sub_cmd_cb_(false, SubCommand::CANCEL);
        usleep(2 * 1000 * 1000);
      }
      enter_on_p2p_state(msg);
      update_state(last_state, AMRStateMachine::ON_P2PNAV);
      break;
    case Message::FOLLOW_PATH:
      if (current_state_ != AMRStateMachine::READY &&
          current_state_ != AMRStateMachine::P2PNAV_WAIT &&
          current_state_ != AMRStateMachine::FOLLOW_PATH_WAIT) {
        return false;
      }
      if (current_state_ == AMRStateMachine::P2PNAV_WAIT) {
        printf("[%s]: Cancel last p2p navigation\n", logger_);
        sub_cmd_cb_(true, SubCommand::CANCEL);
        usleep(2 * 1000 * 1000);
      }
      enter_on_follow_path_state(msg);
      update_state(last_state, AMRStateMachine::ON_FOLLOW_PATH);
      break;
    case Message::WAYPOINT_FOLLOW_PATH:
      if (current_state_ != AMRStateMachine::READY &&
          current_state_ != AMRStateMachine::P2PNAV_WAIT &&
          current_state_ != AMRStateMachine::FOLLOW_PATH_WAIT) {
        return false;
      }
      enter_on_waypoint_follow_path_state(msg);
      update_state(last_state, AMRStateMachine::ON_FOLLOW_PATH);
      break;
    case Message::AE_FINISH:
      if (current_state_ == AMRStateMachine::ON_AE || current_state_ == AMRStateMachine::IDLE) {
        update_state(last_state, AMRStateMachine::LOCALIZATION);
        has_map_ = true;
      }
      break;
    case Message::ME_FINISH:
      if ((current_state_ == AMRStateMachine::IDLE) ||
          (current_state_ == AMRStateMachine::ME_DONE)) {
        update_state(last_state, AMRStateMachine::LOCALIZATION);
        has_map_ = true;
        enter_localization_state();
      }
      break;
    case Message::RELOCALIZATION_PASS:
      if (current_state_ == AMRStateMachine::LOCALIZATION) {
        update_state(last_state, AMRStateMachine::READY);
      }
      break;
    case Message::P2PNAV_FINISH:
      if (current_state_ == AMRStateMachine::ON_P2PNAV) {
        update_state(last_state, AMRStateMachine::READY);
      }
      break;
    case Message::FOLLOW_PATH_FINISH:
      if (current_state_ == AMRStateMachine::ON_FOLLOW_PATH) {
        update_state(last_state, AMRStateMachine::READY);
      }
      break;
    case Message::LOW_POWER:
      if (current_state_ == AMRStateMachine::ON_ME) {
        enter_idle_state();
        update_state(last_state, AMRStateMachine::IDLE);
      } else if (current_state_ == AMRStateMachine::READY ||
                 current_state_ == AMRStateMachine::ON_FOLLOW_PATH ||
                 current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT ||
                 current_state_ == AMRStateMachine::ON_P2PNAV ||
                 current_state_ == AMRStateMachine::P2PNAV_WAIT) {
        enter_ready_state();
        update_state(last_state, AMRStateMachine::READY);
        send_return_charging_message();
      }
      low_power_ = true;
      break;
    case Message::NORMAL_POWER:
      if (current_state_ == AMRStateMachine::LOW_POWER_CHARGING) {
        enter_ready_state();
        update_state(last_state, AMRStateMachine::READY);
      }
      start_charging_cb_(false);
      low_power_ = false;
      break;
    case Message::AMR_EXCEPTION: {
      if (current_state_ == AMRStateMachine::IDLE || current_state_ == AMRStateMachine::ON_ME ||
          current_state_ == AMRStateMachine::ME_DONE ||
          current_state_ == AMRStateMachine::LOCALIZATION) {
        enter_on_error_state();
        update_state(last_state, AMRStateMachine::ON_EXCEPTION);
      } else if (current_state_ == AMRStateMachine::READY ||
                 current_state_ == AMRStateMachine::ON_FOLLOW_PATH ||
                 current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT ||
                 current_state_ == AMRStateMachine::ON_P2PNAV ||
                 current_state_ == AMRStateMachine::P2PNAV_WAIT ||
                 current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
        enter_on_error_state();
        update_state(last_state, AMRStateMachine::ON_ERROR);
      }
      uint8_t error_code = msg.error_code;
      notify_exception_cb_(true, error_code);
    } break;
    case Message::AMR_NORMAL:
      if (current_state_ == AMRStateMachine::ON_ERROR) {
        enter_ready_state();
        update_state(last_state, AMRStateMachine::READY);
        notify_exception_cb_(false, 0);
      }
      break;
    case Message::CANCEL:
      if (current_state_ == AMRStateMachine::ON_ME) {
        enter_idle_state();
        update_state(last_state, AMRStateMachine::IDLE);
      } else if (current_state_ == AMRStateMachine::ON_FOLLOW_PATH ||
                 current_state_ == AMRStateMachine::ON_P2PNAV ||
                 current_state_ == AMRStateMachine::P2PNAV_WAIT ||
                 current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT ||
                 current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
        enter_ready_state();
        update_state(last_state, AMRStateMachine::READY);
      } else {
        return false;
      }
      break;
    case Message::PAUSE:
      printf("[%s]: Message::PAUSE:,current_state_=%d\n", logger_, current_state_);
      if (current_state_ == AMRStateMachine::ON_FOLLOW_PATH) {
        enter_follow_path_wait_state();
        update_state(last_state, AMRStateMachine::FOLLOW_PATH_WAIT);
      } else if (current_state_ == AMRStateMachine::ON_P2PNAV) {
        enter_p2p_wait_state();
        update_state(last_state, AMRStateMachine::P2PNAV_WAIT);
      } else {
        return false;
      }
      break;
    case Message::RESUME:
      if (current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT) {
        sub_cmd_cb_(false, SubCommand::RESUME);
        update_state(last_state, AMRStateMachine::ON_FOLLOW_PATH);
      } else if (current_state_ == AMRStateMachine::P2PNAV_WAIT) {
        sub_cmd_cb_(true, SubCommand::RESUME);
        update_state(last_state, AMRStateMachine::ON_P2PNAV);
      } else {
        return false;
      }
      break;
    case Message::RETURN_CHARGING_FINISH:
      if (current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
        enter_low_power_charging_state();
        update_state(last_state, AMRStateMachine::LOW_POWER_CHARGING);
      } else {
        return false;
      }
      break;
    case Message::RETURN_CHARGING:
      if (current_state_ == AMRStateMachine::READY) {
        enter_on_return_charging_state();
        update_state(last_state, AMRStateMachine::ON_RETURN_CHARGING);
      } else {
        return false;
      }
      break;
    default:
      break;
  }
  printf("[%s]: Finish handle_message\n", logger_);
  return true;
}

void AMRStateMachine::notify_state_machine_changed()
{
  printf("[%s]: send_amr_state_changed:%s\n", logger_, get_current_state().c_str());
  if (send_amr_state_changed_cb_ != nullptr) {
    send_amr_state_changed_cb_(current_state_);
  } else {
    printf("[%s]: send_amr_state_changed_cb_ is null\n", logger_);
  }
}

void AMRStateMachine::handle_failed(const Message & msg)
{
  (void)msg;
}

std::string AMRStateMachine::get_current_state()
{
  return state_to_string(current_state_);
}

void AMRStateMachine::enter_idle_state()
{
  if (current_state_ == AMRStateMachine::ON_ME) {
    bool result;
    slam_command_cb_((uint8_t)Slam_Command::StopMapping, result);
    if (!result) {
      printf("[%s]: Stop mapping failed\n", logger_);
    }
  }
}

void AMRStateMachine::enter_ready_state()
{
  if (send_navigator_cmd_) {
    if ((current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT) ||
        (current_state_ == AMRStateMachine::ON_FOLLOW_PATH)) {
      printf("[%s]: Cancel follow path\n", logger_);
      sub_cmd_cb_(false, SubCommand::CANCEL);
    } else if ((current_state_ == AMRStateMachine::ON_P2PNAV) ||
               (current_state_ == AMRStateMachine::P2PNAV_WAIT)) {
      printf("[%s]: Cancel p2p navigaiton\n", logger_);
      sub_cmd_cb_(true, SubCommand::CANCEL);
    } else if (current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
      printf("[%s]: Cancel return charing station\n", logger_);
      sub_cmd_cb_(true, SubCommand::CANCEL);
    }
    send_navigator_cmd_ = false;
  } else if (send_mapper_cmd_) {
    printf("[%s]: Cancel auto mapping navigaiton\n", logger_);
    // TODO:
    send_mapper_cmd_ = false;
  }
}

void AMRStateMachine::enter_me_done_state()
{
  send_mapper_cmd_ = true;
  save_map();
  send_me_finish_message();
}

void AMRStateMachine::enter_on_me_state()
{
  send_mapper_cmd_ = true;
}

void AMRStateMachine::enter_on_ae_state()
{
  send_mapper_cmd_ = true;
  // TODO:
}

void AMRStateMachine::enter_on_p2p_state(const Message & msg)
{
  send_navigator_cmd_ = true;
  start_p2p_cb_(msg.param);
}

void AMRStateMachine::enter_on_follow_path_state(const Message & msg)
{
  send_navigator_cmd_ = true;
  path_buffer_ = msg.param;
  start_follow_path_cb_(path_buffer_);
}

void AMRStateMachine::enter_on_waypoint_follow_path_state(const Message & msg)
{
  send_navigator_cmd_ = true;
  uint32_t goal_id = msg.goal_id;
  vector<uint32_t> ids;
  ids.assign(msg.ids.begin(), msg.ids.end());
  start_waypoint_follow_path_cb_(goal_id, ids);
}

void AMRStateMachine::enter_on_error_state()
{
  if (send_navigator_cmd_) {
    printf(
        "[%s]: Send cancel nav command when amr is error, state = %d\n", logger_, current_state_);
    if ((current_state_ == AMRStateMachine::FOLLOW_PATH_WAIT) ||
        (current_state_ == AMRStateMachine::ON_FOLLOW_PATH)) {
      sub_cmd_cb_(false, SubCommand::CANCEL);
    } else if ((current_state_ == AMRStateMachine::ON_P2PNAV) ||
               (current_state_ == AMRStateMachine::P2PNAV_WAIT)) {
      sub_cmd_cb_(true, SubCommand::CANCEL);
    } else if (current_state_ == AMRStateMachine::ON_RETURN_CHARGING) {
      sub_cmd_cb_(true, SubCommand::CANCEL);
    } else {
      printf("[%s]: Check the nav state when amr is error\n", logger_);
    }
    send_navigator_cmd_ = false;
  } else if (send_mapper_cmd_) {
    // TODO:
    printf("[%s]: Send cancel mapping command when amr is error\n", logger_);
    send_mapper_cmd_ = false;
  } else {
    printf("[%s]: Nothing to do when amr is error\n", logger_);
  }
}

void AMRStateMachine::enter_follow_path_wait_state()
{
  sub_cmd_cb_(false, SubCommand::PAUSE);
}

void AMRStateMachine::enter_on_return_charging_state()
{
  send_navigator_cmd_ = true;
  return_charging_station();
  start_charging_cb_(true);
}

void AMRStateMachine::enter_p2p_wait_state()
{
  // cancel goal, start goal again when resume
  sub_cmd_cb_(true, SubCommand::PAUSE);
}

void AMRStateMachine::enter_low_power_charging_state()
{
  // TODO:
}

void * AMRStateMachine::get_current_path()
{
  return path_buffer_;
}

void AMRStateMachine::return_charging_station()
{
  navigate_to_charging_cb_();
}

bool AMRStateMachine::check_potential_state(int cmd)
{
  printf("[%s]:Receive cmd:%s, current_state:%s\n", logger_, Command::cmd_to_string(cmd).c_str(),
      get_current_state().c_str());

  if (current_state_ == ON_ERROR) {
    printf("[%s]: check_potential_state(%d,%d) return false\n", logger_, cmd, current_state_);
    return false;
  }

  if (cmd == Command::OTHER) {
    printf("[%s]: check_potential_state(%d,%d) return true\n", logger_, cmd, current_state_);
    return true;
  }
  if (((cmd == Command::AE) || (cmd == Command::ME)) &&
      ((current_state_ == IDLE) || (current_state_ == READY))) {
    printf("[%s]: check_potential_state(%d,%d) return true\n", logger_, cmd, current_state_);
    return true;
  }
  if ((cmd == Command::P2PNAV) &&
      ((current_state_ == READY) || (current_state_ == FOLLOW_PATH_WAIT) ||
          (current_state_ == P2PNAV_WAIT))) {
    printf("[%s]: check_potential_state(%d,%d) return true\n", logger_, cmd, current_state_);
    return true;
  }
  if (((cmd == Command::FOLLOW_PATH) || (cmd == Command::WAYPOINT_FOLLOW_PATH)) &&
      ((current_state_ == READY) || (current_state_ == FOLLOW_PATH_WAIT) ||
          (current_state_ == P2PNAV_WAIT))) {
    printf("[%s]: check_potential_state(%d,%d) return true\n", logger_, cmd, current_state_);
    return true;
  }
  if ((cmd == Command::CHARGING) && ((current_state_ == IDLE) || (current_state_ == READY))) {
    printf("[%s]: check_potential_state(%d,%d) return true\n", logger_, cmd, current_state_);
    return true;
  }
  if ((cmd == Command::SUB_CMD) &&
      ((current_state_ == ON_P2PNAV) || (current_state_ == ON_FOLLOW_PATH) ||
          (current_state_ == P2PNAV_WAIT) || (current_state_ == FOLLOW_PATH_WAIT) ||
          (current_state_ == ON_RETURN_CHARGING) || (current_state_ == ON_ME))) {
    printf("[%s]: check_potential_state(%d,%d) return true\n", logger_, cmd, current_state_);
    return true;
  }

  printf("[%s]: check_potential_state(%d,%d) return false\n", logger_, cmd, current_state_);
  return false;
}

bool AMRStateMachine::check_potential_subcmd_state(uint8_t subcmd)
{
  printf("[%s]: Receive subcmd: [%s]: current_state:%s\n", logger_,
      SubCommand::to_string(subcmd).c_str(), get_current_state().c_str());

  if ((subcmd == SubCommand::PAUSE) &&
      ((current_state_ == P2PNAV_WAIT) || (current_state_ == FOLLOW_PATH_WAIT) ||
          (current_state_ == ON_ME))) {
    printf("[%s]: check_potential_subcmd_state(%d,%d) return false\n", logger_, subcmd,
        current_state_);
    return false;
  }

  if ((subcmd == SubCommand::RESUME) &&
      ((current_state_ == ON_P2PNAV) || (current_state_ == ON_FOLLOW_PATH) ||
          (current_state_ == ON_ME))) {
    printf("[%s]: check_potential_subcmd_state(%d,%d) return false\n", logger_, subcmd,
        current_state_);
    return false;
  }

  if ((subcmd == SubCommand::CANCEL) && (current_state_ == ON_RETURN_CHARGING)) {
    if (low_power_) {
      printf("[%s]: check_potential_subcmd_state(%d,%d) return false when low power\n", logger_,
          subcmd, current_state_);
      return false;
    } else {
      printf("[%s]: check_potential_subcmd_state(%d,%d) return true when normal power\n", logger_,
          subcmd, current_state_);
      return true;
    }
  }

  if ((current_state_ == ON_P2PNAV) || (current_state_ == ON_FOLLOW_PATH) ||
      (current_state_ == P2PNAV_WAIT) || (current_state_ == FOLLOW_PATH_WAIT) ||
      (current_state_ == ON_ME)) {
    printf(
        "[%s]: check_potential_subcmd_state(%d,%d) return true\n", logger_, subcmd, current_state_);
    return true;
  }

  printf("[%s]: check_potential_state(%d,%d) return false\n", logger_, subcmd, current_state_);
  return false;
}

void AMRStateMachine::save_map()
{
  if (slam_command_cb_ == nullptr) {
    printf("[%s]: slam_command_cb_ is nullptr\n", logger_);
    return;
  }

  bool result;
  slam_command_cb_((uint8_t)Slam_Command::SaveMap, result);
  if (!result) {
    printf("[%s]: Save map failed\n", logger_);
  }
}

void AMRStateMachine::enter_localization_state()
{
  bool result = enter_localization_mode();
  if (!result) {
    printf("[%s]: Enter localization mode failed\n", logger_);
    return;
  }

  result = get_relocalization_state();
  if (result) {
    printf("[%s]: Current localization is ready\n", logger_);
    send_relocalization_pass_message();
    return;
  }

  start_rotation();

  while (true) {
    usleep(1000 * 1000);  // sleep 1s
    printf("[%s]: Wait 1 second\n", logger_);
    result = get_relocalization_state();
    if (result) {
      printf("[%s]: Current localization is ready\n", logger_);
      stop_rotation();
      send_relocalization_pass_message();
      return;
    }
  }
}

bool AMRStateMachine::enter_localization_mode()
{
  if (slam_command_cb_ == nullptr) {
    printf("[%s]: slam_command_cb_ is nullptr\n", logger_);
    return false;
  }

  bool result;
  slam_command_cb_((uint8_t)Slam_Command::StartLocalization, result);
  if (result) {
    printf("[%s]: Enter localization mode success\n", logger_);
  } else {
    printf("[%s]: Enter localization mode failed\n", logger_);
  }
  return result;
}

bool AMRStateMachine::get_relocalization_state()
{
  if (slam_command_cb_ == nullptr) {
    printf("[%s]: slam_command_cb_ is nullptr\n", logger_);
    return false;
  }

  bool result;
  slam_command_cb_((uint8_t)Slam_Command::Relocalization, result);
  if (result) {
    printf("[%s]: relocalization is pass\n", logger_);
  } else {
    printf("[%s]: relocalization is failed\n", logger_);
  }
  return result;
}

void AMRStateMachine::start_rotation()
{
  twist_vel twist;
  twist.x = 0;
  twist.y = 0;
  twist.z = ANGULAR_VELOCITY;
  if (publish_twist_cb_ != nullptr) {
    publish_twist_cb_(twist);
  } else {
    printf("[%s]: publish_twist_cb_ is nullptr\n", logger_);
  }
}

void AMRStateMachine::stop_rotation()
{
  twist_vel twist;
  twist.x = 0;
  twist.y = 0;
  twist.z = 0;
  if (publish_twist_cb_ != nullptr) {
    publish_twist_cb_(twist);
  } else {
    printf("[%s]: publish_twist_cb_ is nullptr\n", logger_);
  }
}

void AMRStateMachine::send_me_finish_message()
{
  Message msg;
  msg.type = Message::ME_FINISH;
  queue_.push(msg);
}

void AMRStateMachine::send_me_message()
{
  Message msg;
  msg.type = Message::ME;
  queue_.push(msg);
}

void AMRStateMachine::send_me_completed_message()
{
  Message msg;
  msg.type = Message::ME_COMPLETED;
  queue_.push(msg);
}

void AMRStateMachine::send_relocalization_pass_message()
{
  Message msg;
  msg.type = Message::RELOCALIZATION_PASS;
  queue_.push(msg);
}

void AMRStateMachine::send_return_charging_message()
{
  Message msg;
  msg.type = Message::RETURN_CHARGING;
  queue_.push(msg);
}

void AMRStateMachine::update_state(int last_state, int state)
{
  current_state_ = state;
  if (last_state != current_state_) {
    notify_state_machine_changed();
  }
  printf("[%s]: current state: %s: last state: %s\n", logger_, get_current_state().c_str(),
      state_to_string(last_state).c_str());
}

void AMRStateMachine::activate_node()
{
  if (node_manager_cb_ != nullptr) {
    node_manager_cb_(true);
  } else {
    printf("[%s]: node_manager_cb_ is nullptr\n", logger_);
  }
}

void AMRStateMachine::deactivate_node()
{
  if (node_manager_cb_ != nullptr) {
    node_manager_cb_(false);
  } else {
    printf("[%s]: node_manager_cb_ is nullptr\n", logger_);
  }
}
}  // namespace amr_manager
}  // namespace qrb