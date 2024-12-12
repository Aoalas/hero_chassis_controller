#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

namespace hero_chassis_controller {
    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

    public:
        HeroChassisController() = default;

        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void set_chassis_state(const geometry_msgs::Twist::ConstPtr& msg);

        void cmdVel_Calc(const geometry_msgs::Twist::ConstPtr &msg);

        void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

        void calc_vel() const;

        void updateOdometry();
        tf::TransformBroadcaster odom_broadcaster;
        tf::TransformListener tf_listener;
        tf::StampedTransform transform;
        geometry_msgs::Vector3Stamped global_vel, base_vel;

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

        control_toolbox::Pid front_left_pid_, front_right_pid_, back_left_pid_, back_right_pid_;

        ros::Subscriber joint_state_sub;
        ros::Subscriber cmd_vel_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher odom_pub;

        ros::Time last_time;


    private:
        double wheel_radius_;  // 车轮半径
        double wheel_base_;  //轴距
        double wheel_track_;  //轮距
        std::string speed_mode_;  //速度模式
        double back_left_vel_ = 0;
        double front_left_vel_ = 0;
        double back_right_vel_ = 0;
        double front_right_vel_ = 0;  //速度
        double target_vx_ = 1;
        double target_vy_ = 1;
        double target_omega_ = 0;    //距离方向
        double x = 0.0;
        double y = 0.0;
        double th = 0.0;
        double vx = 0;
        double vy = 0;
        double omega = 0;
        double dx = 0;
        double dy = 0;
        double dth = 0;
        geometry_msgs::Twist a_msg;
        ros::Publisher pub;
    };
}
#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H