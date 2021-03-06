//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMLiftingColumnHardwareInterface_H
#define PROJECT_WMLiftingColumnHardwareInterface_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <pluginlib/class_list_macros.h>

namespace wm_lifting_column_hardware_interface
{
    class WMLiftingColumnHardwareInterface : public hardware_interface::RobotHW {
    public:
        // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
        // Functions
        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override ;
        void read(const ros::Time &time, const ros::Duration &period) override;
        void write(const ros::Time &time, const ros::Duration &period)override;

        // Interface variables
        std::string Name;
        double cmd;
        double pos;
        double vel;
        double eff;
        void StatusCB( std_msgs::Int32 );

    private:
        // Variables
        ros::NodeHandle nh;
        hardware_interface::VelocityJointInterface joint_velocity_interface_;
        hardware_interface::JointStateInterface joint_state_interface_;
        std::string port;
        ros::Publisher CtrlPub;
        ros::Publisher SetPositionPub;
        ros::Subscriber StatSub;

        // Parameters
        std::string mCmdTopic;
        std::string mSetPositionTopic;
        std::string mStateTopic;
        int mMaxCmd;
        bool ready;
        float mMinSpeedUp;
        float mMaxSpeedUp;
        float mMinSpeedDown;
        float mMaxSpeedDown;
        float mMaxHeight;
        int mResolution;
        double posBuffer;
        int mBumperTimeCounter;
        double mFilteredCmd;
    };
}
#endif //PROJECT_WMLiftingColumnHardwareInterface_H
