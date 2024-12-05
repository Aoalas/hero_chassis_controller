#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>

namespace hero_chassis_controller {
    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {

        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        controller_nh.getParam("wheel_radius", wheel_radius_);
        controller_nh.getParam("chassis_width", chassis_width_);
        controller_nh.getParam("chassis_length", chassis_length_);
        controller_nh.getParam("wheelbase", wheel_base_);
        controller_nh.getParam("wheel_track", wheel_track_);

        //初始化 PID 控制器
        front_left_pid_.init(ros::NodeHandle(controller_nh, "front_left_pid"));
        front_right_pid_.init(ros::NodeHandle(controller_nh, "front_right_pid"));
        back_left_pid_.init(ros::NodeHandle(controller_nh, "back_left_pid"));
        back_right_pid_.init(ros::NodeHandle(controller_nh, "back_right_pid"));

        // 订阅动作指令话题
        joint_state_sub = root_nh.subscribe("/joint_states", 10, &HeroChassisController::jointStateCallback, this);
        //订阅当前速度
        cmd_vel_sub = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

        last_cmd_time_ = ros::Time::now();
        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

//        double timeout = 0.5; // 如果超过 0.5 秒未收到新的 /cmd_vel 消息，则停止运动
//        if ((time - last_cmd_time_).toSec() > timeout) {
//            desired_vx_ = 0.0;
//            desired_vy_ = 0.0;
//            desired_omega_ = 0.0;
//            last_cmd_time_ = time;
//        }
        // 获得底盘整体的目标速度
        double vx = desired_vx_;
        double vy = desired_vy_;
        double omega = desired_omega_;
//        ROS_INFO("计算轮子目标速度: %f %f %f", vx, vy, omega);

//      获得底盘当前实际速度
        double now_back_left_vel_ = back_left_vel_;
        double now_front_left_vel_ = front_left_vel_;
        double now_back_right_vel_ = back_right_vel_;
        double now_front_right_vel_ = front_right_vel_;;
//        逆运动学
        double front_left_target = (vx - vy - (chassis_width_ + chassis_length_) * omega) / wheel_radius_;
        double front_right_target = (vx + vy + (chassis_width_ + chassis_length_) * omega) / wheel_radius_;
        double back_left_target = (vx + vy - (chassis_width_ + chassis_length_) * omega) / wheel_radius_;
        double back_right_target = (vx - vy + (chassis_width_ + chassis_length_) * omega) / wheel_radius_;

        // 读取当前轮子速度
//        double front_left_effort = front_left_joint_.getEffort();
//        double front_right_effort = front_right_joint_.getEffort();
//        double back_left_effort = back_left_joint_.getEffort();
//        double back_right_effort = back_right_joint_.getEffort();
//        ROS_INFO("now vel: %f %f %f %f", front_left_effort, front_right_effort, back_left_effort, back_right_effort);

        // 计算 PID 控制命令
        double front_left_cmd = front_left_pid_.computeCommand(front_left_target - now_front_left_vel_, period);
        double front_right_cmd = front_right_pid_.computeCommand(front_right_target - now_front_right_vel_, period);
        double back_left_cmd = back_left_pid_.computeCommand(back_left_target - now_back_left_vel_, period);
        double back_right_cmd = back_right_pid_.computeCommand(back_right_target - now_back_right_vel_, period);

//        ROS_INFO("cmd: %f %f %f %f", front_left_cmd, front_right_cmd, back_left_cmd, back_right_cmd);
        front_left_joint_.setCommand(front_left_cmd);
        front_right_joint_.setCommand(front_right_cmd);
        back_left_joint_.setCommand(back_left_cmd);
        back_right_joint_.setCommand(back_right_cmd);
    }

//  读取/vmd_vel上的目标速度
    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        desired_vx_ = msg->linear.x;    // x轴线速度（前进）
        desired_vy_ = msg->linear.y;    // y轴线速度（侧移）
        desired_omega_ = msg->angular.z;    // z轴角速度（旋转）
        ROS_INFO("keyboard cmd: %f %f %f", desired_vx_, desired_vy_, desired_omega_);
        last_cmd_time_ = ros::Time::now(); // 更新最新命令时间
    }

//  读取当前四个轮子速度
    void HeroChassisController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
        back_left_vel_ = msg->velocity[0];
        front_left_vel_ = msg->velocity[1];
        back_right_vel_ = msg->velocity[2];
        front_right_vel_ = msg->velocity[3];
    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller