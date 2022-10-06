#include "roger_hand_ros/hand_driver.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roger_hand_ros/hand_control.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hand_driver");
  ros::NodeHandle n("~");

  HandDriver hand_driver = HandDriver(&n);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (!hand_driver.Active()) {       // Hand is not active
      hand_driver.EnableHand();    // Check and initialise if possible
    } else {                           // Hand is active
      hand_driver.PublishHandState();  // Publish hand state
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}