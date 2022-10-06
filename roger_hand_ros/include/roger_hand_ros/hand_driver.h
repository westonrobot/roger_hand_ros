#ifndef HAND_DRIVER_H
#define HAND_DRIVER_H

// C++
#include <unistd.h>
#include <stdio.h>
#include <string>

// ROS
#include "ros/ros.h"
#include "roger_hand_ros/hand_control.h"
#include <roger_hand_msgs/ampere_feedback.h>
#include <roger_hand_msgs/clear_error.h>
#include <roger_hand_msgs/finger_pose.h>
#include <roger_hand_msgs/hand_enable.h>
#include <roger_hand_msgs/hand_pose.h>
#include <roger_hand_msgs/hand_state.h>

class HandDriver {
 public:
  HandDriver(ros::NodeHandle *nh);
  ~HandDriver();

  bool EnableHand();
  bool Active();
  bool PublishHandState();

 private:
  // ----- ROS Node Parameters -----
  std::string port_name_;
  // ----- Internal Variables -----
  ros::NodeHandle *nh_;
  int fd_ = -1;
  bool is_active_ = false;
  int ampere_threshold_ = 800;
  bool ampere_feedback_ = false;
  PortInfo_t portinfo_ = {BAUDRATE, 8, 2, 0, 1, 0};
  int is_finger_enabled_[6] = {0};
  int pos_dst_[6] = {0};
  int pos_real_[6] = {0};
  int temp_[6] = {0};
  int ampere_[6] = {0};
  int err_[6] = {0};
  // ----- Published Messages-----
  // ----- Subscribers & Publishers & Services-----
  ros::Publisher hand_state_pub;

  ros::ServiceServer set_finger_pose_server_;
  ros::ServiceServer set_hand_pose_server_;
  ros::ServiceServer set_hand_enable_server_;
  ros::ServiceServer set_ampere_feedback_server_;
  ros::ServiceServer clear_hand_error_server_;
  // ----- Callbacks -----
  bool set_finger_pose_cb(roger_hand_msgs::finger_pose::Request &req,
                          roger_hand_msgs::finger_pose::Response &res);

  bool set_hand_pose_cb(roger_hand_msgs::hand_pose::Request &req,
                        roger_hand_msgs::hand_pose::Response &res);

  bool set_ampere_feedback_cb(roger_hand_msgs::ampere_feedback::Request &req,
                              roger_hand_msgs::ampere_feedback::Response &res);

  bool set_hand_enable_cb(roger_hand_msgs::hand_enable::Request &req,
                          roger_hand_msgs::hand_enable::Response &res);

  bool clear_hand_error_cb(roger_hand_msgs::clear_error::Request &req,
                           roger_hand_msgs::clear_error::Response &res);

  bool ReadParameters();
  bool SetupInterfaces();
  void DisableHand();
  bool IsPortAccessible(std::string port_name);
};

#endif /* HAND_DRIVER_H */
