#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <controller_interface/controller_base.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/PidConfigConfig.h>
#include <effort_controllers/joint_effort_controller.h>
#include <forward_command_controller/forward_command_controller.h>

namespace hero_chassis_controller {

    HeroChassisController::HeroChassisController()
            : state_(0) {}

    bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                     ros::NodeHandle& controller_nh)
    {
//        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
//        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
//        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
//        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        try {
            front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
            front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
            back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
            back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
            ROS_INFO("Successfully initialized joint handles.");
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM("Failed to get joint handle: " << e.what());
            return false;
        }
        // 初始化 PID 控制器


        // 初始化动态重配置服务器
        dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<hero_chassis_controller::PidConfigConfig>>(controller_nh);

        // 设置动态参数回调
        dynamic_reconfigure::Server<hero_chassis_controller::PidConfigConfig>::CallbackType f;
        f = [this](auto && PH1, auto && PH2) { dynamicReconfigureCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); };
//        f = boost::bind(&HeroChassisController::dynamicReconfigureCallback, this, _1, _2);
        dynamic_reconfigure_server_->setCallback(f);

        return true;
    }

    bool HeroChassisController::initRequest(hardware_interface::RobotHW* robot_hw,
                                            ros::NodeHandle& root_nh,
                                            ros::NodeHandle& controller_nh,
                                            controller_interface::ControllerBase::ClaimedResources& resources)
    {
        // 如果没有特殊的硬件初始化需求，可以简单返回 true
        return true;
    }

    void HeroChassisController::starting(const ros::Time& time)
    {
        // 在控制器开始时初始化所需的内容
        // 你可以在此处初始化 PID 等控制器参数
        ros::NodeHandle nh;
        float p_front_left_,i_front_left_,d_front_left_;
        float p_front_right_,i_front_right_,d_front_right_;
        float p_back_left_,i_back_left_,d_back_left_;
        float p_back_right_,i_back_right_,d_back_right_;
        // 从参数服务器加载 PID 参数
        nh.getParam("controller/hero_chassis_controller/p_front_left", p_front_left_);  // 默认值为 1.0
        nh.getParam("controller/hero_chassis_controller/i_front_left", i_front_left_);  // 默认值为 0.0
        nh.getParam("controller/hero_chassis_controller/d_front_left", d_front_left_);  // 默认值为 0.0

        nh.getParam("controller/hero_chassis_controller/p_front_right", p_front_right_);
        nh.getParam("controller/hero_chassis_controller/i_front_right", i_front_right_);
        nh.getParam("controller/hero_chassis_controller/d_front_right", d_front_right_);

        nh.getParam("controller/hero_chassis_controller/p_back_left", p_back_left_);
        nh.getParam("controller/hero_chassis_controller/i_back_left", i_back_left_);
        nh.getParam("controller/hero_chassis_controller/d_back_left", d_back_left_);

        nh.getParam("controller/hero_chassis_controller/p_back_right", p_back_right_);
        nh.getParam("controller/hero_chassis_controller/i_back_right", i_back_right_);
        nh.getParam("controller/hero_chassis_controller/d_back_right", d_back_right_);

        pid_front_left_.initPid(p_front_left_, i_front_left_, d_front_left_, 10.0, -10.0);
        pid_front_right_.initPid(p_front_right_, i_front_right_, d_front_right_, 10.0, -10.0);
        pid_back_left_.initPid(p_back_left_, i_back_left_, d_back_left_, 10.0, -10.0);
        pid_back_right_.initPid(p_back_right_, i_back_right_, d_back_right_, 10.0, -10.0);
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        double error[4] = { 0.1, 0.1, 0.1, 0.1 };

        front_left_joint_.setCommand(pid_front_left_.computeCommand(error[0], period));
        front_right_joint_.setCommand(pid_front_right_.computeCommand(error[1], period));
        back_left_joint_.setCommand(pid_back_left_.computeCommand(error[2], period));
        back_right_joint_.setCommand(pid_back_right_.computeCommand(error[3], period));
    }

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)

}  // namespace hero_chassis_controller

