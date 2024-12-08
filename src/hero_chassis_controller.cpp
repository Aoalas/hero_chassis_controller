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

        //获取参数配置文件里面的参数
        controller_nh.getParam("wheel_radius", wheel_radius_);  //轮半径
        controller_nh.getParam("wheel_base", wheel_base_);   //轴距
        controller_nh.getParam("wheel_track", wheel_track_);  //轮距

        //初始化 PID 控制器
        front_left_pid_.init(ros::NodeHandle(controller_nh, "front_left_pid"));
        front_right_pid_.init(ros::NodeHandle(controller_nh, "front_right_pid"));
        back_left_pid_.init(ros::NodeHandle(controller_nh, "back_left_pid"));
        back_right_pid_.init(ros::NodeHandle(controller_nh, "back_right_pid"));

        // 订阅动作指令话题
        joint_state_sub = root_nh.subscribe("/joint_states", 10, &HeroChassisController::jointStateCallback, this);
        //订阅当前速度
        cmd_vel_sub = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

        odom_pub = root_nh.advertise<nav_msgs::Odometry>("/odom", 50);

        last_cmd_time_ = ros::Time::now();  // 更新最新命令时间
        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

        // 获得整体的目标速度
        double vx = goal_vx_;
        double vy = goal_vy_;
        double omega = goal_omega_;
        //ROS_INFO("计算轮子目标速度: %f %f %f", vx, vy, omega);

        //获得各个轮子的当前速度
        front_left_vel_ = front_right_joint_.getVelocity();
        front_right_vel_ = back_right_joint_.getVelocity();
        back_left_vel_ = front_left_joint_.getVelocity();
        back_right_vel_ = back_left_joint_.getVelocity();
        //四轮驱动的逆运动学公式
        double front_left_target = (vx - vy - (wheel_track_ + wheel_base_) * omega) / wheel_radius_;
        double front_right_target = (vx + vy + (wheel_track_ + wheel_base_) * omega) / wheel_radius_;
        double back_left_target = (vx + vy - (wheel_track_ + wheel_base_) * omega) / wheel_radius_;
        double back_right_target = (vx - vy + (wheel_track_ + wheel_base_) * omega) / wheel_radius_;

        //  PID 控制命令
        double front_left_cmd = front_left_pid_.computeCommand(front_left_target - front_left_vel_, period);
        double front_right_cmd = front_right_pid_.computeCommand(front_right_target - front_right_vel_, period);
        double back_left_cmd = back_left_pid_.computeCommand(back_left_target - back_left_vel_, period);
        double back_right_cmd = back_right_pid_.computeCommand(back_right_target - back_right_vel_, period);

        front_left_joint_.setCommand(front_left_cmd);
        front_right_joint_.setCommand(front_right_cmd);
        back_left_joint_.setCommand(back_left_cmd);
        back_right_joint_.setCommand(back_right_cmd);
    }

    //读取当前四个轮子速度
    void HeroChassisController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
        back_left_vel_ = msg->velocity[0];
        front_left_vel_ = msg->velocity[1];
        back_right_vel_ = msg->velocity[2];
        front_right_vel_ = msg->velocity[3];
    }


    //读取/cmd_vel上的速度
    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        goal_vx_ = msg->linear.x;    // 整个底盘的x轴线速度（前进）
        goal_vy_ = msg->linear.y;    // 整个底盘的y轴线速度（侧移）
        goal_omega_ = msg->angular.z;    // 整个底盘的z轴角速度（旋转）

        last_cmd_time_ = ros::Time::now(); // 更新时间
    }

//    void HeroChassisController::updateOdometry()
//    {
//        ros::Time current_time, last_time;
//        current_time = ros::Time::now();
//        last_time = ros::Time::now();
//
//        //compute odometry in a typical way given the velocities of the robot
//        double dt = (current_time - last_time).toSec();
//        double delta_x = (od_vx * cos(th) - od_vy * sin(th)) * dt;
//        double delta_y = (od_vx * sin(th) + od_vy * cos(th)) * dt;
//        double delta_th = od_vth * dt;
//
//        x += delta_x;
//        y += delta_y;
//        th += delta_th;
//
//
//        //since all odometry is 6DOF we'll need a quaternion created from yaw
//        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
//
//        //first, we'll publish the transform over tf
//        geometry_msgs::TransformStamped odom_trans;
//        odom_trans.header.stamp = current_time;
//        odom_trans.header.frame_id = "odom";
//        odom_trans.child_frame_id = "base_link";
//
//        odom_trans.transform.translation.x = x;
//        odom_trans.transform.translation.y = y;
//        odom_trans.transform.translation.z = 0.0;
//        odom_trans.transform.rotation = odom_quat;
//
//        // send the transform
//        odom_broadcaster.sendTransform(odom_trans);
//
//        nav_msgs::Odometry odom_data{};
//        odom_data.header.stamp = current_time;
//        odom_data.header.frame_id = "odom";
//
//        odom_data.pose.pose.position.x = x;
//        odom_data.pose.pose.position.y = y;
//        odom_data.pose.pose.position.z = 0.0;
//        odom_data.pose.pose.orientation = odom_quat;
//
//        odom_data.child_frame_id = "base_link";
//        odom_data.twist.twist.linear.x = od_vx;
//        odom_data.twist.twist.linear.y = od_vy;
//        odom_data.twist.twist.angular.z = od_vth;
//
//        odom_pub.publish(odom_data);
//        last_time = current_time;
//    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller