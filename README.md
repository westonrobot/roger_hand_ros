# Roger Robot Hand ROS

## About
This package provides a ROS interface to Weston Robot's roger robot hand end-effector. Please first check if your device and environment are supported by this package.

**Tested Environments**:

* Architecture: x86_64
* OS: Ubuntu 18.04/20.04
* ROS: Melodic/Noetic

**Supported Peripherals**:

* [Roger Robot Hand](https://docs.westonrobot.net/periph_user_guide/periph_user_guide.html)

## Building the package
```
$ cd <your-catkin-ws>/src
$ git clone https://github.com/westonrobot/roger_hand_ros.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

## Nodes
### hand_driver_node
| Published Topic | Type                        | Description                  |
| --------------- | --------------------------- | ---------------------------- |
| `~/hand_state`  | roger_hand_msgs::hand_state | Outputs hand's current state |


| Service                 | Type                             | Description                                 |
| ----------------------- | -------------------------------- | ------------------------------------------- |
| `~/set_finger_pose`     | roger_hand_msgs::finger_pose     | Set pose of one [joint](#joint-ids)         |
| `~/set_hand_pose`       | roger_hand_msgs::hand_pose       | Set pose of entire hand (6 joints)          |
| `~/clear_hand_error`    | roger_hand_msgs::clear_error     | Clear any errors raised                     |
| `~/set_ampere_feedback` | roger_hand_msgs::ampere_feedback | Enable current feedback and threshold level |
| `~/set_hand_enable`     | roger_hand_msgs::hand_enable     | (Dis/En)able hand operation                 |


## Example Usage
**_You may need to change runtime parameters by editing the corresponding launch file_**

1. Launch the roger hand server
``` bash
$ roslaunch roger_hand_bringup roger_hand_server.launch
```

2. Check current state of hand
``` bash
$ rostopic echo /roger_hand/hand_driver/hand_state
```

3. Set thumb facing up
``` bash
$ rosservice call /roger_hand/hand_driver/set_finger_pose 1 0
```

4. Reset hand to default state
``` bash
$ rosservice call /roger_hand/set_hand_pose [2000,300,2000,2000,2000,2000]
```

5. Clear errors
   * call this service when:
     * current state of hand shows 1 under hand_err
     * hand fails to respond to single finger / full hand control
``` bash
$ rosservice call /roger_hand/clear_hand_error
```

6. Set ampere(current) feedback and threshold level (Defaults: true, 800)
``` bash
$ rosservice call /roger_hand/set_ampere_feedback true 800
```

7. Disable hand
``` bash
$ rosservice call /roger_hand/set_hand_enable false
```

## Appendix

### Joint IDs

| Parameter | Joint            |
| --------- | ---------------- |
| `1`       | Thumb            |
| `2`       | Thumb (Rotation) |
| `3`       | Index            |
| `4`       | Middle           |
| `5`       | Ring             |
| `6`       | Little           |

### Port permission
If you get the error "... permission denied ..." when trying to open the port, you need to grant access of the port to your user accout.
```
$ sudo usermod -a -G dialout $USER
```
You need to re-login to get the change to take effect.
