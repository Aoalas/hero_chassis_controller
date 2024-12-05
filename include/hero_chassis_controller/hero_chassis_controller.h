#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>

namespace hero_chassis_controller {
    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

    public:
        HeroChassisController() = default;

        ~HeroChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

        void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

        control_toolbox::Pid front_left_pid_, front_right_pid_, back_left_pid_, back_right_pid_;

        ros::Subscriber joint_state_sub;
        ros::Subscriber cmd_vel_sub;

        ros::Time last_cmd_time_;

    private:
        double wheel_radius_;  // 车轮半径
        double chassis_width_;  // 底盘宽
        double chassis_length_;  // 底盘长
        double wheel_base_;  //轴距
        double wheel_track_;  //轮距
        double back_left_vel_ = 0;
        double front_left_vel_ = 0;
        double back_right_vel_ = 0;
        double front_right_vel_ = 0;  //速度
        double desired_vx_ = 0;
        double desired_vy_ = 0;
        double desired_omega_ = 0;    //距离方向

    };
}
#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H