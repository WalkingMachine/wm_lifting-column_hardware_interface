//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMRobotiqHardwareInterface_H
#define PROJECT_WMRobotiqHardwareInterface_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <robotiq_85_msgs/GripperCmd.h>
#include <robotiq_85_msgs/GripperStat.h>
#include <pluginlib/class_list_macros.h>

namespace wm_robotiq_hardware_interface
{
    class WMRobotiqHardwareInterface : public hardware_interface::RobotHW {
    public:
        // << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
        // Functions
        virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
        virtual void read(const ros::Time &time, const ros::Duration &period);
        virtual void write(const ros::Time &time, const ros::Duration &period);

        // Interface variables
        std::string Name;
        double cmd;
        double pos;
        double vel;
        double eff;
        void StatusCB( robotiq_85_msgs::GripperStatConstPtr );

    private:
        // Variables
        ros::NodeHandle nh;
        static hardware_interface::PositionJointInterface joint_position_interface_;
        static hardware_interface::JointStateInterface joint_state_interface_;
        std::string port;
        ros::Publisher GripperCtrlPub;
        ros::Subscriber GripperStatSub;

    };
}
#endif //PROJECT_WMRobotiqHardwareInterface_H
