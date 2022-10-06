#include "roger_hand_ros/hand_driver.h"

HandDriver::HandDriver(ros::NodeHandle *nh) : nh_(nh) {
  if (!HandDriver::ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  if (!HandDriver::SetupInterfaces()) {
    ROS_ERROR("Could not setup ros interfaces");
    ros::shutdown();
  }
}

HandDriver::~HandDriver() {
  if (is_active_) {
    HandDriver::DisableHand();
  }
}

bool HandDriver::ReadParameters() {
  // Get parameters
  if (!nh_->getParam("port_name", port_name_)) {
    ROS_ERROR("Please set the hand's serial port");
    return false;
  }

  ROS_INFO("--- Parameters loaded are ---");

  ROS_INFO_STREAM("port_name: " << port_name_);

  ROS_INFO("-----------------------------");

  return true;
}

bool HandDriver::SetupInterfaces() {
  set_finger_pose_server_ = nh_->advertiseService(
      "set_finger_pose", &HandDriver::set_finger_pose_cb, this);
  set_hand_pose_server_ = nh_->advertiseService(
      "set_hand_pose", &HandDriver::set_hand_pose_cb, this);
  set_ampere_feedback_server_ = nh_->advertiseService(
      "set_ampere_feedback", &HandDriver::set_ampere_feedback_cb, this);
  set_hand_enable_server_ = nh_->advertiseService(
      "set_hand_enable", &HandDriver::set_hand_enable_cb, this);
  clear_hand_error_server_ = nh_->advertiseService(
      "clear_hand_error", &HandDriver::clear_hand_error_cb, this);

  hand_state_pub =
      nh_->advertise<roger_hand_msgs::hand_state>("hand_state", 10, true);
  return true;
}

bool HandDriver::IsPortAccessible(std::string port_name) {
  if (access(port_name.c_str(), 0) == 0) {
    return true;
  } else {
    return false;
  }
}

bool HandDriver::EnableHand() {
  if (IsPortAccessible(port_name_)) {
    fd_ = SerialOpen(port_name_.c_str());
    if (fd_ < 0) {
      ROS_ERROR_STREAM("Failed to open port:" << port_name_);
      is_active_ = false;
      return false;
    } else {
      ROS_DEBUG_STREAM("Successfully open port:" << port_name_);
      SerialSet(fd_, &portinfo_);
      is_active_ = true;
      ampere_feedback_ = true;
      enable_hand(fd_);
      for (int index = 0; index < 6; index++) {
        is_finger_enabled_[index] = true;
      }
      return true;
    }
  } else {
    is_active_ = false;
    return false;
  }
}

void HandDriver::DisableHand() {
  SerialClose(fd_);
  is_active_ = false;
}

bool HandDriver::Active() { return is_active_; }

bool HandDriver::set_finger_pose_cb(
    roger_hand_msgs::finger_pose::Request &req,
    roger_hand_msgs::finger_pose::Response &res) {
  if (req.index == 1 || req.index == 2 || req.index == 3 || req.index == 4 ||
      req.index == 5 || req.index == 6) {
    sleep_time(1);
    int index = req.index;
    pos_dst_[index - 1] = req.pose;
    int len = WritePos(fd_, index, pos_dst_[index - 1], pos_real_[index - 1],
                       temp_[index - 1], ampere_[index - 1], err_[index - 1]);
    if (len >= 0)
      res.set_ok = true;
    else
      res.set_ok = false;
  } else {
    ROS_ERROR("The index is wrong!");
    res.set_ok = false;
  }

  return true;
}

bool HandDriver::set_hand_pose_cb(roger_hand_msgs::hand_pose::Request &req,
                                  roger_hand_msgs::hand_pose::Response &res) {
  if (req.pose.size() > 6) {
    ROS_ERROR("Too Many Arguments!");
    res.set_ok = false;
    return true;
  }
  if (req.pose.size() < 6) {
    ROS_ERROR("Too Few Arguments!");
    res.set_ok = false;
    return true;
  }
  if (req.pose.size() == 6) {
    for (int index = 0; index < 6; index++) {
      pos_dst_[index] = (int)req.pose.data()[index];
    }
  }

  res.set_ok = true;
  sleep_time(1);
  int len;
  for (int index = 1; index <= 6; index++) {
    len = WritePos(fd_, index, pos_dst_[index - 1], pos_real_[index - 1],
                   temp_[index - 1], ampere_[index - 1], err_[index - 1]);
    if (len < 0) res.set_ok = false;
  }
  return true;
}

bool HandDriver::set_ampere_feedback_cb(
    roger_hand_msgs::ampere_feedback::Request &req,
    roger_hand_msgs::ampere_feedback::Response &res) {
  ampere_threshold_ = req.ampere_threshold;
  ampere_feedback_ = req.ampere_feedback;
  res.set_ok = true;
  return true;
}

bool HandDriver::set_hand_enable_cb(
    roger_hand_msgs::hand_enable::Request &req,
    roger_hand_msgs::hand_enable::Response &res) {
  if (req.hand_work) {
    enable_hand(fd_);
    for (int index = 0; index < 6; index++) {
      is_finger_enabled_[index] = true;
    }
  } else {
    unable_hand(fd_);
    for (int index = 0; index < 6; index++) {
      is_finger_enabled_[index] = false;
    }
  }
  res.set_ok = true;
  return true;
}

bool HandDriver::clear_hand_error_cb(
    roger_hand_msgs::clear_error::Request &req,
    roger_hand_msgs::clear_error::Response &res) {
  clearErr_hand(fd_);
  res.set_ok = true;
  return true;
}

bool HandDriver::PublishHandState() {
  if (IsPortAccessible(port_name_)) {
    roger_hand_msgs::hand_state hand_state_;
    std::vector<int> hand_pose_;
    std::vector<int> hand_temp_;
    std::vector<int> hand_ampere_;
    std::vector<int> hand_err_;
    std::vector<int> finger_enable_;

    for (int index = 1; index <= 6; index++) {
      int pos_real_, temp_, ampere_, err_;
      ReadState(fd_, index, pos_real_, temp_, ampere_, err_);
      if (ampere_feedback_) {
        if (ampere_ > ampere_threshold_) {
          if (is_finger_enabled_[index - 1])
            StopWorking(fd_, index, pos_real_, temp_, ampere_, err_);
          is_finger_enabled_[index - 1] = false;
        } else {
          // if (!is_finger_enabled_[index-1])
          //   StartWorking(fd_, index, pos_real_, temp_, ampere_, err_);
          // is_finger_enabled_[index-1] = true;
        }
      }

      hand_pose_.push_back(pos_real_);
      hand_temp_.push_back(temp_);
      hand_ampere_.push_back(ampere_);
      hand_err_.push_back(err_);
      finger_enable_.push_back(is_finger_enabled_[index - 1]);
    }
    hand_state_.finger_enable = finger_enable_;
    hand_state_.hand_pose = hand_pose_;
    hand_state_.hand_temp = hand_temp_;
    hand_state_.hand_ampere = hand_ampere_;
    hand_state_.hand_err = hand_err_;
    hand_state_.hand_ampere_threshold = ampere_threshold_;
    hand_state_.hand_ampere_feedback = ampere_feedback_;
    hand_state_pub.publish(hand_state_);
    return true;
  } else {
    if (is_active_) {
      DisableHand();
    }
    ROS_WARN_STREAM("Port: " << port_name_ << " is not accessible!!!");
    return false;
  }
}