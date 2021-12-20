# ZGT Robotic Hand ROS

## Build

OS: Ubuntu 18.04 LTS
ROS: Melodic

```
$ cd <your-catkin-ws>/src
$ git clone https://github.com/westonrobot/zgt_hand_ros.git
$ cd ..
$ catkin_make
```

## Usage

1. 启动棉手服务端
2. roslaunch zgt_hand_ros zgt_hand_server.launch
3. 客户端指令
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

Note:  If you get the error "... permission denied ..." when trying to open the port, you need to grant access of the port to your user accout:

```
$ sudo usermod -a -G dialout $USER
```
You need to re-login to get the change to take effect.
