# Hero_chassis_controller 

Written by *ZhuolinLiu*(刘焯林)

~~RM is not your entire life.~~

## Overview

This is a controller for a hero_chassis_robot from this website: 

https://github.com/YoujianWu/rm_description_for_task.

And this is my assignment for **final-task**.

This controller use PID to control the motion of this robot.

The hero_chassis_controller package has been tested under [ROS](http://www.ros.org/) Noetic on respectively 20.04.

**Keywords:** ROS ,ros_control, sleepless, helpless

### dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org/) 
- `roscpp`
- `roslint`
- `dynamic_reconfigure`
- `hardware_interface`
- `controller_interface`
- `pluginlib`
- `std_msgs`
- `tf`
- `forward_command_controller`

### About me

Author:  *ZhuolinLiu*(刘焯林)

Github ID:  Aoalas

Address: 2874257768@qq.com

## Usage

Just Download it and put it in your catkin workspace, don't forget to **catkin build**.

After you roslaunch the hero robot, type this code in your terminal👇

```
roslaunch hero_chassis_controller hero_chassis_controller_pid.launch 
```

Now the PID function is ready, you can modify the PID parameter to control it.

For example, rosrun your rqt_reconfigure then you can see the PID operation interface for each wheel.

## Files in package

### Config files

Config file config

- **controllers.yaml** Params of hero_chassis_controller and joint_state_controller.

  

### Launch files

- **hero_chassis_controller_pid.launch :** Activate the PID for hero_chassis_robot.

  

### Other files

- **cfg/PidConfig.cfg** **:**  The file for PID, you don't need to modify it.
- The Folder named **src** and **include** contains the code for the function, you can check it out and get some details in it.

## Additional controller (Keyboard)

I made a keyboard controller for hero robot, I push it to other repository, if can help you to control the robot with keyboard, the usage is written in the README.md in this repository.

Here is the link ↓↓↓

https://github.com/Aoalas/keyboard_controller

## Gratitude list

Thanks to GDUT for letting me know about this amazing team called DynamicX.😎

Thanks to happy_cat senior, Ciler senior, and all the other seniors for their guidances to us.😊

Thanks to my pro peers who remind my weakness and help me to urge myself to work harder.😭

Thanks to my body for allowing me to persevere in many situations when I stay up so late and stay up all night.😇

Thanks to those pros extremely who will show mercy to me. ( If I successfully join the team ) 😘

