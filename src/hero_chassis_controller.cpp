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
        controller_nh.getParam("wheel_base", wheel_base_);  //轴距
        controller_nh.getParam("wheel_track", wheel_track_);  //轮距
        controller_nh.getParam("speed_mode", speed_mode_);  //速度模式

        //初始化 PID 控制器
        front_left_pid_.init(ros::NodeHandle(controller_nh, "front_left_pid"));
        front_right_pid_.init(ros::NodeHandle(controller_nh, "front_right_pid"));
        back_left_pid_.init(ros::NodeHandle(controller_nh, "back_left_pid"));
        back_right_pid_.init(ros::NodeHandle(controller_nh, "back_right_pid"));

        // 订阅动作指令话题
        joint_state_sub = root_nh.subscribe("/joint_states", 10, &HeroChassisController::jointStateCallback, this);
        //速度发布者
//        cmd_vel_pub = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        //订阅当前速度
        cmd_vel_sub= root_nh.subscribe("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);
        //创建发布者，将数据发以  nav_msgs/Odometry  发布到"/odom"上
        odom_pub = root_nh.advertise<nav_msgs::Odometry>("/odom", 30);

        last_cmd_time_ = ros::Time::now();  // 更新时间
        return true;
    }


    //读取轮子的速度(不知道为什么，要是删掉这个函数，程序就不能运行了)
    void HeroChassisController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
        back_left_vel_ = msg->velocity[0];
        front_left_vel_ = msg->velocity[1];
        back_right_vel_ = msg->velocity[2];
        front_right_vel_ = msg->velocity[3];
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
        ros::NodeHandle nh;
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        front_left_vel_ = front_right_joint_.getVelocity();
        front_right_vel_ = back_right_joint_.getVelocity();
        back_left_vel_ = front_left_joint_.getVelocity();
        back_right_vel_ = back_left_joint_.getVelocity();
        //四轮驱动的逆运动学公式
        double front_left_target =
                (target_vx_ - target_vy_ - (wheel_track_ + wheel_base_) * target_omega_)/wheel_radius_;
        double front_right_target =
                (target_vx_ + target_vy_ + (wheel_track_ + wheel_base_) * target_omega_)/wheel_radius_;
        double back_left_target =
                (target_vx_ + target_vy_ - (wheel_track_ + wheel_base_) * target_omega_)/wheel_radius_;
        double back_right_target =
                (target_vx_ - target_vy_ + (wheel_track_ + wheel_base_) * target_omega_)/wheel_radius_;

        //  PID 控制命令
        double front_left_cmd = front_left_pid_.computeCommand(front_left_target - front_left_vel_, period);
        double front_right_cmd = front_right_pid_.computeCommand(front_right_target - front_right_vel_, period);
        double back_left_cmd = back_left_pid_.computeCommand(back_left_target - back_left_vel_, period);
        double back_right_cmd = back_right_pid_.computeCommand(back_right_target - back_right_vel_, period);

        front_left_joint_.setCommand(front_left_cmd);
        front_right_joint_.setCommand(front_right_cmd);
        back_left_joint_.setCommand(back_left_cmd);
        back_right_joint_.setCommand(back_right_cmd);
        //初始化msg
        a_msg.linear.x = 0.1;
        a_msg.linear.y = 0.1;
        a_msg.linear.z = 0;
        a_msg.angular.x = 0;
        a_msg.angular.y = 0;
        a_msg.angular.z = 0;
        pub.publish(a_msg); // 发布消息
    }


//    //读取/cmd_vel上的速度
//    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
//        vx = msg->linear.x;    // 整个底盘的x轴线速度（前进）
//        vy = msg->linear.y;    // 整个底盘的y轴线速度（侧移）
//        omega = msg->angular.z;    // 整个底盘的z轴角速度（旋转）
//        //以目前从/cmd_vel上读取的速度来计算当前每个轮子的速度
//        double front_left_now = (vx - vy - (wheel_track_ + wheel_base_) * omega);
//        double front_right_now = (vx + vy + (wheel_track_ + wheel_base_) * omega);
//        double back_left_now = (vx + vy - (wheel_track_ + wheel_base_) * omega);
//        double back_right_now = (vx - vy + (wheel_track_ + wheel_base_) * omega);
//        ROS_INFO("Now speed:%f,%f,%f,%f", front_left_now, front_right_now, back_left_now, back_right_now);
//        last_cmd_time_ = ros::Time::now(); // 更新时间
//    }
    // 获取当前cmd_vel上的速度指令
    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        // 获取当前机器人与世界坐标系的变换
        if (speed_mode_ == "global") {
            // 如果速度模式是全局坐标系(global)，则需要转换速度到底盘中，以控制底盘
            tf::StampedTransform transform;
            tf_listener.lookupTransform("base_link", "odom", ros::Time(0), transform);

            // 进行坐标转换，将全局速度转换为底盘坐标系下的速度
            tf::Vector3 global_velocity(vx, vy, omega);
            tf::Vector3 local_velocity = transform.inverse() * global_velocity;
            // 更新目标速度
            vx = local_velocity.x();
            vy = local_velocity.y();
            omega = local_velocity.z();
        } else {
            // 如果速度模式是底盘坐标系(local)，则直接传入速度
            vx = msg->linear.x;    // x轴线速度（前进）
            vy = msg->linear.y;    // y轴线速度（侧移）
            omega = msg->angular.z;    // 角速度（旋转）
        }
        //以目前从/cmd_vel上读取的速度来计算当前每个轮子的速度
        double front_left_now = (vx - vy - (wheel_track_ + wheel_base_) * omega);
        double front_right_now = (vx + vy + (wheel_track_ + wheel_base_) * omega);
        double back_left_now = (vx + vy - (wheel_track_ + wheel_base_) * omega);
        double back_right_now = (vx - vy + (wheel_track_ + wheel_base_) * omega);
        ROS_INFO("Now speed:  %f  ,  %f  ,  %f  ,  %f  ", front_left_now, front_right_now, back_left_now, back_right_now);
        last_cmd_time_ = ros::Time::now();  // 更新时间
    }

    void HeroChassisController::updateOdometry(const sensor_msgs::JointState::ConstPtr &msg) {
        //根据车轮的速度计算里程计
        double dt = (ros::Time::now() - last_cmd_time_).toSec();   // 根据时间差计算dt
        last_cmd_time_ = ros::Time::now();              //记录last_cmd_time_用于计算dt

        //获取各个车轮的速度
        //  back_left_vel_ = msg->velocity[0];
        //  front_left_vel_ = msg->velocity[1];
        //  back_right_vel_ = msg->velocity[2];
        //  front_right_vel_ = msg->velocity[3];
        double v1 = msg->velocity[3];       //front_right
        double v2 = msg->velocity[2];       //back_right
        double v3 = msg->velocity[1];       //front_left
        double v4 = msg->velocity[0];       //back_left

        //使用微分思想进行计算与累加
        dx = wheel_radius_ * 0.25 * (v1 + v2 + v3 + v4);
        dy = wheel_radius_ * 0.25 * (v3 + v2 - v1 - v4);
        dth = wheel_radius_ * (v1 + v2 - v3 - v4) / ((wheel_track_ / 2 + wheel_base_ / 2) * 4);

        x += dx * cos(th) * dt - dy * sin(th) * dt;
        y += dx * sin(th) * dt + dy * cos(th) * dt;
        th += dth * dt;

        //构造 nav_msgs::Odometry 消息来储存各种信息，并且将th转换为四元数
        nav_msgs::Odometry odom_Data{};
        geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(th);

        odom_Data.header.stamp = ros::Time::now();
        odom_Data.header.frame_id = "odom";

        odom_Data.pose.pose.position.x = x;
        odom_Data.pose.pose.position.y = y;
        odom_Data.pose.pose.position.z = 0.0;
        odom_Data.pose.pose.orientation = odom_quaternion;

        odom_Data.child_frame_id = "base_link";
        odom_Data.twist.twist.linear.x = dx;
        odom_Data.twist.twist.linear.y = dy;
        odom_Data.twist.twist.angular.z = dth;

        odom_pub.publish(odom_Data);    //发布


        //从"odom"到"base_link"的坐标变换关系
        geometry_msgs::TransformStamped odom_Trans;
        odom_Trans.header.stamp = ros::Time::now();
        odom_Trans.header.frame_id = "odom";
        odom_Trans.child_frame_id = "base_link";

        odom_Trans.transform.translation.x = x;
        odom_Trans.transform.translation.y = y;
        odom_Trans.transform.translation.z = 0;
        odom_Trans.transform.rotation = odom_quaternion;
        //发送tf变换
        odom_broadcaster.sendTransform(odom_Trans);
    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller