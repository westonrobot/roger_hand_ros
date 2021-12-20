#include "zgt_hand_ros/hand_driver.h"
#include "zgt_hand_ros/hand_control.h"

handDriver::handDriver() {}

handDriver::~handDriver() { close_hand_(); }

bool handDriver::hand_detect_(std::string file_name) {
  const char *file_name_ = file_name.c_str();
  if (access(file_name_, 0) == 0) {
    return true;
  } else {
    return false;
  }
}

bool handDriver::open_hand_(std::string file_name) {
  if (hand_detect_(file_name)) {
    fd = SerialOpen(file_name.c_str());
    if (fd < 0) {
      ROS_ERROR("Open File Failed!");
      open_hand = false;
      return false;
    } else {
      SerialSet(fd, &portinfo);
      open_hand = true;
      ampere_feedback = true;
      enable_hand(fd);
      for (int index_ = 0; index_ < 6; index_++)
        finger_work[index_] = true;
      return true;
    }
  }
  return false;

  //   ROS_INFO_STREAM_ONCE(open_hand);
}

void handDriver::close_hand_() {
  SerialClose(fd);
  open_hand = false;
}

bool handDriver::handDriverInit(ros::NodeHandle &root_nh) {
  nh_ = root_nh;
  set_finger_pose_server_ = nh_.advertiseService(
      "set_finger_pose", &handDriver::set_finger_pose_cb, this);
  set_hand_pose_server_ = nh_.advertiseService(
      "set_hand_pose", &handDriver::set_hand_pose_cb, this);
  set_ampere_feedback_server_ = nh_.advertiseService(
      "set_ampere_feedback", &handDriver::set_ampere_feedback_cb, this);
  set_hand_enable_server_ = nh_.advertiseService(
      "set_hand_enable", &handDriver::set_hand_enable_cb, this);
  clear_hand_error_server_ = nh_.advertiseService(
      "clear_hand_error", &handDriver::clear_hand_error_cb, this);

  hand_state_pub =
      nh_.advertise<zgt_hand_ros::hand_state>("hand_state", 10, true);
  return true;
}

bool handDriver::set_finger_pose_cb(zgt_hand_ros::finger_pose::Request &req,
                                    zgt_hand_ros::finger_pose::Response &res) {

  if (req.index == 1 || req.index == 2 || req.index == 3 || req.index == 4 ||
      req.index == 5 || req.index == 6) {
      sleep_time(1);
      int index_ = req.index;
      pos_dst[index_ - 1] = req.pose;
      int len = WritePos(fd, index_, pos_dst[index_ - 1], pos_real[index_ - 1],
                         temp[index_ - 1], ampere[index_ - 1], err[index_ - 1]);
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

bool handDriver::set_hand_pose_cb(zgt_hand_ros::hand_pose::Request &req,
                                  zgt_hand_ros::hand_pose::Response &res) {

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
    for (int index_ = 0; index_ < 6; index_++) {
      pos_dst[index_] = (int)req.pose.data()[index_];
    }
  }

  res.set_ok = true;
  sleep_time(1);
  int len;
  for (int index_ = 1; index_ <= 6; index_++) {
    len = WritePos(fd, index_, pos_dst[index_ - 1], pos_real[index_ - 1],
                   temp[index_ - 1], ampere[index_ - 1], err[index_ - 1]);
    if (len < 0)
      res.set_ok = false;
  }
  return true;
}

bool handDriver::set_ampere_feedback_cb(
    zgt_hand_ros::ampere_feedback::Request &req,
    zgt_hand_ros::ampere_feedback::Response &res) {
  ampere_threshold = req.ampere_threshold;
  ampere_feedback = req.ampere_feedback;
  res.set_ok = true;
  return true;
}

bool handDriver::set_hand_enable_cb(zgt_hand_ros::hand_enable::Request &req,
                                    zgt_hand_ros::hand_enable::Response &res) {
  if (req.hand_work) {
    enable_hand(fd);
    for (int index_ = 0; index_ < 6; index_++)
      finger_work[index_] = true;
  } else {
    unable_hand(fd);
    for (int index_ = 0; index_ < 6; index_++)
      finger_work[index_] = true;
  }
  res.set_ok = true;
  return true;
}

bool handDriver::clear_hand_error_cb(zgt_hand_ros::clear_error::Request &req,
                                     zgt_hand_ros::clear_error::Response &res) {

  clearErr_hand(fd);
  res.set_ok = true;
  return true;
}

bool handDriver::hand_state_pub_() {
  zgt_hand_ros::hand_state hand_state_;
  std::vector<int> hand_pose_;
  std::vector<int> hand_temp_;
  std::vector<int> hand_ampere_;
  std::vector<int> hand_err_;
  std::vector<int> finger_enable_;

  for (int index_ = 1; index_ <= 6; index_++) {
    int pos_real, temp, ampere, err;
    ReadState(fd, index_, pos_real, temp, ampere, err);
    if (ampere_feedback) {
      if (ampere > ampere_threshold) {
        if (finger_work[index_ - 1])
          StopWorking(fd, index_, pos_real, temp, ampere, err);
        finger_work[index_ - 1] = false;
      } else {
        // if (!finger_work[index_-1])
        //   StartWorking(fd, index_, pos_real, temp, ampere, err);
        // finger_work[index_-1] = true;
      }
    }

    hand_pose_.push_back(pos_real);
    hand_temp_.push_back(temp);
    hand_ampere_.push_back(ampere);
    hand_err_.push_back(err);
    finger_enable_.push_back(finger_work[index_ - 1]);
  }
  hand_state_.finger_enable = finger_enable_;
  hand_state_.hand_pose = hand_pose_;
  hand_state_.hand_temp = hand_temp_;
  hand_state_.hand_ampere = hand_ampere_;
  hand_state_.hand_err = hand_err_;
  hand_state_.hand_ampere_threshold = ampere_threshold;
  hand_state_.hand_ampere_feedback = ampere_feedback;
  hand_state_pub.publish(hand_state_);
  return true;
}