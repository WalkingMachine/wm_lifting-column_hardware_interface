//
// Created by philippe on 03/05/17.
//

#include "WMLiftingColumnHardwareInterface.h"
#include <nodelet/nodelet.h>
#include <iostream>

namespace wm_lifting_column_hardware_interface {


    hardware_interface::PositionJointInterface WMLiftingColumnHardwareInterface::joint_position_interface_;
    hardware_interface::JointStateInterface    WMLiftingColumnHardwareInterface::joint_state_interface_;

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>

    bool WMLiftingColumnHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
        using namespace hardware_interface;
        std::cout << "INIT LIFTINGCOLUMN\n";

        // Get parameters
        std::vector<std::string> Joints;
        if (!robot_hw_nh.getParam("joints", Joints)) { return false; }
        Name = Joints[0];
        robot_hw_nh.getParam("port", port);

        // Initialise interface variables
        cmd = 0;
        pos = 0;
        vel = 0;
        eff = 0;

        // Register interfaces
        joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
        joint_position_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_position_interface_);

        // advertise publisher
        GripperCtrlPub = root_nh.advertise<std_msgs::Int32>( "column/cmd", 1 );
        //GripperStatSub.
        GripperStatSub = root_nh.subscribe( "column/state", 1, &WMLiftingColumnHardwareInterface::StatusCB, this);

        return true;
    }

    void WMLiftingColumnHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
    }

    void WMLiftingColumnHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        std_msgs::Int32 msg;
        msg.data = cmd;
        GripperCtrlPub.publish( msg );
    }

    void WMLiftingColumnHardwareInterface::StatusCB( std_msgs::Int32 msg ){
        pos = msg.data;
    }

}


PLUGINLIB_EXPORT_CLASS( wm_lifting_column_hardware_interface::WMLiftingColumnHardwareInterface, hardware_interface::RobotHW)