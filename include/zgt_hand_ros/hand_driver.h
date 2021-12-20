#ifndef _HAND_DRIVER_H_
#define _HAND_DRIVER_H_

#include "ros/ros.h"
#include "zgt_hand_ros/hand_control.h"
#include <stdio.h>
#include <unistd.h>
#include <zgt_hand_ros/ampere_feedback.h>
#include <zgt_hand_ros/clear_error.h>
#include <zgt_hand_ros/finger_pose.h>
#include <zgt_hand_ros/hand_enable.h>
#include <zgt_hand_ros/hand_pose.h>
#include <zgt_hand_ros/hand_state.h>

class handDriver {
public:
  handDriver();
  ~handDriver();
  bool handDriverInit(ros::NodeHandle &root_nh);

  bool set_finger_pose_cb(zgt_hand_ros::finger_pose::Request &req,
                        zgt_hand_ros::finger_pose::Response &res);

  bool set_hand_pose_cb(zgt_hand_ros::hand_pose::Request &req,
                        zgt_hand_ros::hand_pose::Response &res);

  bool set_ampere_feedback_cb(zgt_hand_ros::ampere_feedback::Request &req,
                              zgt_hand_ros::ampere_feedback::Response &res);

  bool set_hand_enable_cb(zgt_hand_ros::hand_enable::Request &req,
                          zgt_hand_ros::hand_enable::Response &res);

  bool clear_hand_error_cb(zgt_hand_ros::clear_error::Request &req,
                           zgt_hand_ros::clear_error::Response &res);
  bool hand_state_pub_();

  bool open_hand_(std::string file_name);
  void close_hand_();
  bool hand_detect_(std::string file_name);

  ros::NodeHandle nh_;
  ros::ServiceServer set_finger_pose_server_;
  ros::ServiceServer set_hand_pose_server_;
  ros::ServiceServer set_hand_enable_server_;
  ros::ServiceServer set_ampere_feedback_server_;
  ros::ServiceServer clear_hand_error_server_;

  ros::Publisher hand_state_pub;

  int fd = -1;
  int ampere_threshold = 800;
  bool open_hand = false;
  bool ampere_feedback = false;
  PortInfo_t portinfo = {BAUDRATE, 8, 2, 0, 1, 0};
  int finger_work[6] = {0}, pos_dst[6] = {0}, pos_real[6] = {0}, temp[6] = {0},
      ampere[6] = {0}, err[6] = {0};
};

#endif
