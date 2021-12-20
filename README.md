# zgt_hand_ros
ZGT 灵巧手ROS控制
=============

## ROS安装说明

安装版本：Kinetic(1.12.14)
系统版本: ubuntu(16.04.5)
安装说明详见：http://wiki.ros.org/kinetic/Installation/Ubuntu

## 例程安装说明
1: 将zgt_hand_ros包放到ros空间src文件中
2: 编译灵巧手源码
　　　catkin_make
3: 提示编译成功

## 例程使用说明

1: 启动棉手服务端
2: roslaunch zgt_hand_ros zgt_hand_server.launch
3: 客户端指令
   a.查看灵巧手状态
   rostopic echo /zgt_hand/hand_state
   b.发布单个电机运动控制指令（参数：电机编号1~6,运动控制指令0~2000）
    rosservice call /zgt_hand/set_finger_pose 1 2000
   c.发布灵巧手运动控制指令（参数：一组电机运动控制指令0~2000）
   rosservice call /zgt_hand/set_hand_pose [2000,300,2000,2000,2000,2000] 
　  d.清除灵巧手异常（无参数）
   rosservice call /zgt_hand/clear_hand_error 
   e.设置灵巧手电流反馈（参数：是否开启电流反馈　电流反馈阈值设置）
   rosservice call /zgt_hand/set_ampere_feedback true 800
   rosservice call /zgt_hand/set_ampere_feedback false 0
　  f.设置灵巧手工作/停止（参数：工作/停止）
   rosservice call /zgt_hand/set_hand_enable true
   rosservice call /zgt_hand/set_hand_enable false

## 备注
 请自行安装串口驱动.
 串口文件是ttyUSB*,请自行添加使用权限（chmod, chown),具体名称请自行查看,可在launch文件中修改.

