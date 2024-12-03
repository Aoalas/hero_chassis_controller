#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/PidConfigConfig.h>
#include <controller_interface/controller_base.h>
#include <effort_controllers/joint_effort_controller.h>
#include <forward_command_controller/forward_command_controller.h>
namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::ControllerBase {
    public:
        HeroChassisController();

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &controller_nh);

        void update(const ros::Time &time, const ros::Duration &period) override;

        void setPID(double p, double i, double d);

        void dynamicReconfigureCallback(hero_chassis_controller::PidConfigConfig &config, uint32_t level);
//        {
//            ROS_INFO_STREAM("Reconfigure Request: p = " << config.p << ", i = " << config.i << ", d = " << config.d);
//
//            pid_front_left_.initPid(config.p, config.i, config.d, 10.0, -10.0);  // 初始化前左轮 PID
//            pid_front_right_.initPid(config.p, config.i, config.d, 10.0, -10.0); // 初始化前右轮 PID
//            pid_back_left_.initPid(config.p, config.i, config.d, 10.0, -10.0);   // 初始化后左轮 PID
//            pid_back_right_.initPid(config.p, config.i, config.d, 10.0, -10.0);  // 初始化后右轮 PID
//        }

        // 实现 initRequest 函数
        virtual bool initRequest(hardware_interface::RobotHW *robot_hw,
                                 ros::NodeHandle &root_nh,
                                 ros::NodeHandle &controller_nh,
                                 controller_interface::ControllerBase::ClaimedResources &resources) override;

        // 实现 starting 函数
        virtual void starting(const ros::Time &time) override;
        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle back_left_joint_;
        hardware_interface::JointHandle back_right_joint_;

        std::shared_ptr<dynamic_reconfigure::Server<hero_chassis_controller::PidConfigConfig>> dynamic_reconfigure_server_;
    private:

        // 四个 PID 控制器，每个控制一个轮子
        control_toolbox::Pid pid_front_left_;
        control_toolbox::Pid pid_front_right_;
        control_toolbox::Pid pid_back_left_;
        control_toolbox::Pid pid_back_right_;

        ros::Time last_change_;
        int state_;
    };

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_H