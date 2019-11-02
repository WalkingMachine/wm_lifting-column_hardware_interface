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
        ready = false;

        // Register interfaces
        joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
        joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_velocity_interface_);

        // Get the parameters
        robot_hw_nh.param<std::string>("cmd_topic", mCmdTopic, "column/cmd");
        robot_hw_nh.param<std::string>("set_position_topic", mSetPositionTopic, "column/set_position");
        robot_hw_nh.param<std::string>("position_topic", mStateTopic, "column/position");
        robot_hw_nh.param<int>("cmd_max", mMaxCmd, 255);
        robot_hw_nh.param<float>("speed_max_up", mMaxSpeedUp, 0.0279503106f);
        robot_hw_nh.param<float>("speed_min_up", mMinSpeedUp, 0.0093243243f);
        robot_hw_nh.param<float>("speed_max_down", mMaxSpeedDown, 0.02957988f);
        robot_hw_nh.param<float>("speed_min_down", mMinSpeedDown, 0.010952381f);
        robot_hw_nh.param<float>("max_height", mMaxHeight, 0.69);
        robot_hw_nh.param<int>("resolution", mResolution, 9560);

        // advertise publishers
        CtrlPub = root_nh.advertise<std_msgs::Int32>( mCmdTopic, 1 );
        SetPositionPub = root_nh.advertise<std_msgs::Int32>( mSetPositionTopic, 1 );
        //GripperStatSub.
        StatSub = root_nh.subscribe( mStateTopic, 1, &WMLiftingColumnHardwareInterface::StatusCB, this);

        while (!ready) {
            ROS_WARN("Lifting column is not ready!: Waiting for messages in %s", mStateTopic.c_str());
            sleep(1);
        }

        return true;
    }

    void WMLiftingColumnHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        vel = (posBuffer-pos)/period.toSec();
        pos = posBuffer;

        // Check if we reached a bumper by compairing command with velocity
        if (vel < 0.0001 && vel > 0.0001 && (cmd < -mMinSpeedDown || cmd > mMinSpeedUp) ){
            // Apply a couter to filter out response times.
            mBumperTimeCounter ++;
            if (mBumperTimeCounter == 20){
                // Send the new position
                std_msgs::Int32 msg;
                msg.data = cmd > 0 ? mResolution : 0;
                SetPositionPub.publish( msg );
            }
        } else {
            mBumperTimeCounter = 0;

            // Apply limitations on the position
            std_msgs::Int32 msg;
            if (pos > mMaxHeight){
                pos = mResolution;
                msg.data = pos;
                SetPositionPub.publish( msg );
            } else if (pos < 0){
                pos = 0;
                msg.data = pos;
                SetPositionPub.publish( msg );
            }
        }
    }

    void WMLiftingColumnHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {

        double tempCmd;
        // Apply the speed limits
        if (cmd > mMaxSpeedUp){
            tempCmd = mMaxCmd;
        } else if (cmd > mMinSpeedUp){
            tempCmd *= mMaxCmd/mMaxSpeedUp;
        } else if (cmd < -mMaxSpeedDown){
            tempCmd = -mMaxCmd;
        } else if (cmd < -mMinSpeedDown){
            tempCmd *= mMaxCmd/mMaxSpeedDown;
        }

        mFilteredCmd += (tempCmd-vel)/1;

        if (mFilteredCmd/mMaxCmd > -mMinSpeedDown && mFilteredCmd/mMaxCmd < mMinSpeedUp) {
            mFilteredCmd = 0;
        }


        // Send the mFilteredCmd
        std_msgs::Int32 msg;
        msg.data = mFilteredCmd;
        CtrlPub.publish( msg );
    }

    void WMLiftingColumnHardwareInterface::StatusCB( std_msgs::Int32 msg ) {
        // Ajust resolution
        posBuffer = msg.data*mMaxHeight/mResolution;
        ready = true;
    }

}


PLUGINLIB_EXPORT_CLASS( wm_lifting_column_hardware_interface::WMLiftingColumnHardwareInterface, hardware_interface::RobotHW)
