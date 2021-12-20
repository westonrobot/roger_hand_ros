#include "zgt_hand_ros/hand_driver.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "zgt_hand_ros/hand_control.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "hand_driver");
  ros::NodeHandle n;

  std::string port_name;
  bool ifget = ros::param::get("port_name",port_name);
  if(!ifget)
  {
    ROS_INFO("Please enter the port name.");
    return 0;
  }
  handDriver hand_Driver;
  hand_Driver.handDriverInit(n);
	ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (!hand_Driver.hand_detect_(port_name)) 
		{
			if(hand_Driver.open_hand)hand_Driver.close_hand_();
			loop_rate.sleep();
      continue;
    }
    if (!hand_Driver.open_hand) {
      ros::Rate loop_rate_(0.5);
      loop_rate_.sleep();
      hand_Driver.open_hand_(port_name);
    }
    hand_Driver.hand_state_pub_();
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}