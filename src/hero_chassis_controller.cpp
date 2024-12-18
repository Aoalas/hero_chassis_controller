//Created by Aoalas
//最终考核代码，其中ROS_INFO已被注释
//关于键盘控制的程序，我新开了一个软件包，在另一个仓库内，详细请看README.md里的内容

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
        controller_nh.getParam("wheel_radius", wheel_radius);  //轮半径
        controller_nh.getParam("wheel_base", wheel_base);  //轴距
        controller_nh.getParam("wheel_track", wheel_track);  //轮距
        controller_nh.getParam("speed_mode", speed_mode);  //速度模式

        //初始化 PID 控制器
        front_left_pid_.init(ros::NodeHandle(controller_nh, "front_left_pid"));
        front_right_pid_.init(ros::NodeHandle(controller_nh, "front_right_pid"));
        back_left_pid_.init(ros::NodeHandle(controller_nh, "back_left_pid"));
        back_right_pid_.init(ros::NodeHandle(controller_nh, "back_right_pid"));

        //订阅/cmd_vel上的速度
        cmd_vel_sub = root_nh.subscribe("/cmd_vel", 10, &HeroChassisController::cmdVel_Calc, this);
        //创建发布者，将数据发以  nav_msgs/Odometry  发布到"/odom"上
        odom_pub = root_nh.advertise<nav_msgs::Odometry>("/odom", 30);

        last_time_ = ros::Time::now();  // 更新时间
        return true;
    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {

        ros::Time current_time = time;
        front_left_vel_ = wheel_radius * front_left_joint_.getVelocity();
        front_right_vel_ = wheel_radius * front_right_joint_.getVelocity();
        back_left_vel_ = wheel_radius * back_left_joint_.getVelocity();
        back_right_vel_ = wheel_radius * back_right_joint_.getVelocity();
        //四轮驱动的逆运动学公式
        double front_left_target =
                (target_vx_ - target_vy_ - (wheel_track + wheel_base) * target_omega_) / wheel_radius;
        double front_right_target =
                (target_vx_ + target_vy_ + (wheel_track + wheel_base) * target_omega_) / wheel_radius;
        double back_left_target =
                (target_vx_ + target_vy_ - (wheel_track + wheel_base) * target_omega_) / wheel_radius;
        double back_right_target =
                (target_vx_ - target_vy_ + (wheel_track + wheel_base) * target_omega_) / wheel_radius;

        //  PID 控制命令
        double front_left_cmd = front_left_pid_.computeCommand(front_left_target - front_left_vel_, period);
        double front_right_cmd = front_right_pid_.computeCommand(front_right_target - front_right_vel_, period);
        double back_left_cmd = back_left_pid_.computeCommand(back_left_target - back_left_vel_, period);
        double back_right_cmd = back_right_pid_.computeCommand(back_right_target - back_right_vel_, period);

        front_left_joint_.setCommand(front_left_cmd);
        front_right_joint_.setCommand(front_right_cmd);
        back_left_joint_.setCommand(back_left_cmd);
        back_right_joint_.setCommand(back_right_cmd);


        //因为没办法在cmd_vel中直接读取机器人的整体速度，所以就只能读取各车轮速度，再传建发布者来发布上去了(
        ros::NodeHandle nh;
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        double v1 = wheel_radius * front_left_joint_.getVelocity();
        double v2 = wheel_radius * front_right_joint_.getVelocity();
        double v3 = wheel_radius * back_left_joint_.getVelocity();
        double v4 = wheel_radius * back_right_joint_.getVelocity();
        vx = 0.5 * (v3 + v4);
        vy = 0.5 * (v3 - v1);
        omega = 0.5 * (v2 - v3) / ((wheel_track / 2 + wheel_base / 2));
        //消息信息设置
        a_msg.linear.x = vx;
        a_msg.linear.y = vy;
        a_msg.linear.z = 0;
        a_msg.angular.x = 0;
        a_msg.angular.y = 0;
        a_msg.angular.z = omega;

        pub.publish(a_msg); // 发布消息

        updateOdometry();

        last_time_ = current_time;
    }

    // 获取当前cmd_vel上的速度
    void HeroChassisController::cmdVel_Calc(const geometry_msgs::Twist::ConstPtr &msg) {
        // 获取当前机器人与世界坐标系的变换
        if (speed_mode == "global") {
            // 如果速度模式是全局坐标系(global)，则需要转换速度到底盘中，以控制底盘
            global_vel.header.frame_id = "odom";
            global_vel.header.stamp = ros::Time(0);
            global_vel.vector.x = target_vx_;
            global_vel.vector.y = target_vy_;
            global_vel.vector.z = 0.0;
            tf_listener.waitForTransform("base_link", "odom", ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("base_link", "odom", ros::Time(0), transform);
            tf_listener.transformVector("base_link", global_vel, base_vel);
            target_vx_ = base_vel.vector.x;
            target_vy_ = base_vel.vector.y;
        } else {
            // 如果速度模式是底盘坐标系(local)，则直接传入速度
            vx = msg->linear.x;    // x轴线速度（前进）
            vy = msg->linear.y;    // y轴线速度（侧移）
            omega = msg->angular.z;    // 角速度（旋转）
        }
        //以目前从/cmd_vel上读取的速度来计算当前每个轮子的角速度（题目5）
        //输出各轮子速度(注释化ROS_INFO时记得把四个轮子速度的计算也注释，不然会报错)

//        double front_left_now = (vx - vy - (wheel_track + wheel_base) * omega);
//        double front_right_now = (vx + vy + (wheel_track + wheel_base) * omega);
//        double back_left_now = (vx + vy - (wheel_track + wheel_base) * omega);
//        double back_right_now = (vx - vy + (wheel_track + wheel_base) * omega);
//        ROS_INFO("FL: %f ,FR: %f ,BL: %f ,BR: %f ", front_left_now, front_right_now, back_left_now,
//                 back_right_now);

        last_time_ = ros::Time::now();  // 更新时间
    }


    void HeroChassisController::updateOdometry() {
        //里程计
        double dt = (ros::Time::now() - last_time_).toSec();   // 根据时间差计算dt
        last_time_ = ros::Time::now();              //记录last_time_，用于下一次计算dt

        double v1 = wheel_radius * front_left_joint_.getVelocity();
        double v2 = wheel_radius * front_right_joint_.getVelocity();
        double v3 = wheel_radius * back_left_joint_.getVelocity();
        double v4 = wheel_radius * back_right_joint_.getVelocity();

        //使用微分思想进行计算与累加
        // 查到有两种公式，不知道哪种更准确，但是效果似乎一样
        dx = 0.5 * (v3 + v4);
        dy = 0.5 * (v3 - v1);
        dth = 0.5 * (v2 - v3) / ((wheel_track / 2 + wheel_base / 2));

        //dx = 0.25 * (v1 + v2 + v3 + v4);
        //dy = 0.25 * (-v1 + v2 + v3 - v4);
        //dth = 0.25 * (-v1 + v2 - v3 + v4) / ((wheel_track / 2 + wheel_base / 2));

        x += dx * cos(th) * dt - dy * sin(th) * dt;
        y += dx * sin(th) * dt + dy * cos(th) * dt;
        th += dth * dt;


        //输出里程计（题目6）
//        ROS_INFO("X_way :%f, Y_way:%f.", x, y);

        //构造 nav_msgs::Odometry 消息来储存各种信息，并且将th转换为四元数

        nav_msgs::Odometry odom_Data{};
        tf2::Quaternion q;
        q.setRPY(0, 0, th);
        geometry_msgs::Quaternion odom_quaternion = toMsg(q);

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
        static ros::Time last_tf_publish_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        //设置发布频率，默认的速度太快，容易报错unknown_publisher
        if ((current_time - last_tf_publish_time).toSec() >= 0.02) {// 50Hz 发布频率
            odom_broadcaster.sendTransform(odom_Trans);
            last_tf_publish_time = current_time;  // 更新上次发布时间
        }

        odom_Data.header.stamp = ros::Time::now();
        odom_Data.header.frame_id = "odom";
        odom_Data.child_frame_id = "base_link";

        odom_Data.pose.pose.position.x = x;
        odom_Data.pose.pose.position.y = y;
        odom_Data.pose.pose.position.z = 0.0;
        odom_Data.pose.pose.orientation = odom_quaternion;
        odom_Data.twist.twist.linear.x = vx;
        odom_Data.twist.twist.linear.y = vy;
        odom_Data.twist.twist.angular.z = dth;
        //发布
        odom_pub.publish(odom_Data);

    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller
