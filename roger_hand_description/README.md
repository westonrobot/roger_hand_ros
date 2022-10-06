# roger_hand_description
roger 灵巧手ROS URDF模型
=============

## ROS安装说明

安装版本：Noetic
系统版本: ubuntu(20.04)
安装说明详见：http://wiki.ros.org/kinetic/Installation/Ubuntu

## 例程安装说明
1: 将roger_hand_description包放到ros空间src文件中
2: 编译灵巧手源码
　　　catkin_make
3: 提示编译成功

## 例程使用说明

## 在RViz环境试用:
   roslaunch roger_hand_description display.launch
## 在Gazebo环境调用模型
   roslaunch roger_hand_gazebo.launch
   
## 备注
   在URDF模型中给的Joint的接口的指令是电机丝杆的位移指令0-10mm；
   如果需要在仿真中用到实际的电机的指令，则需要进行指令转化，即0-2000对应电机的全行程，直接把全行程等分2000份。

