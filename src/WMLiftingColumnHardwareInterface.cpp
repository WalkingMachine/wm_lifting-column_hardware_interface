//
// Created by philippe on 03/05/17.
//

#include "WMLiftingColumnHardwareInterface.h"
#include <nodelet/nodelet.h>
#include <iostream>

namespace wm_lifting_column_hardware_interface {

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
        posBuffer = 0;
        vel = 0;
        eff = 0;

        // Register interfaces
        joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
        joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_velocity_interface_);

        // Get the parameters
        robot_hw_nh.param<std::string>("cmd_topic", mCmdTopic, "column/cmd");
        robot_hw_nh.param<std::string>("state_topic", mStateTopic, "column/state");
        robot_hw_nh.param<int>("cmd_max", mMaxCmd, 255);
        robot_hw_nh.param<float>("speed_max_up", mMaxSpeedUp, 0.0279503106f);
        robot_hw_nh.param<float>("speed_min_up", mMinSpeedUp, 0.0093243243f);
        robot_hw_nh.param<float>("speed_max_down", mMaxSpeedDown, 0.02957988f);
        robot_hw_nh.param<float>("speed_min_down", mMinSpeedDown, 0.010952381f);
        robot_hw_nh.param<float>("max_height", mMaxHeight, 69);
        robot_hw_nh.param<int>("resolution", mResolution, 2387);

        // advertise publisher
        CtrlPub = root_nh.advertise<std_msgs::Int32>( "column/cmd", 1 );
        //GripperStatSub.
        StatSub = root_nh.subscribe( "column/state", 1, &WMLiftingColumnHardwareInterface::StatusCB, this);

        return true;
    }

    void WMLiftingColumnHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        pos = posBuffer;
    }

    void WMLiftingColumnHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {

        // Apply the speed limits
        if (cmd > mMaxSpeedUp){
            cmd = mMaxSpeedUp;
        } else if (cmd > mMinSpeedUp){
            cmd *= mMaxCmd/mMaxSpeedUp;
        } else if (cmd < -mMaxSpeedDown){
            cmd = -mMaxSpeedDown;
        } else if (cmd < -mMinSpeedDown){
            cmd *= mMaxCmd/mMaxSpeedDown;
        } else {
            cmd = 0;
        }

        // Send the command
        std_msgs::Int32 msg;
        msg.data = (int)cmd;
        CtrlPub.publish( msg );
    }

    void WMLiftingColumnHardwareInterface::StatusCB( std_msgs::Int32 msg ) {
        // Ajust resolution
        posBuffer = msg.data*mMaxHeight/mResolution;
    }

}


PLUGINLIB_EXPORT_CLASS( wm_lifting_column_hardware_interface::WMLiftingColumnHardwareInterface, hardware_interface::RobotHW)