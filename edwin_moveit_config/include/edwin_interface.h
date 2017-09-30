#ifndef __EDWIN_INTERFACE_H__
#define __EDWIN_INTERFACE_H__

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <string>

#include "st_arm.h"
#include "irl/arm_cmd.h"

class EdwinInterface: public hardware_interface::RobotHW{
	private:
		STArm st;

		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;

		double cmd[N_JOINTS];
		double pos[N_JOINTS];
		double vel[N_JOINTS];
		double eff[N_JOINTS];

		ros::NodeHandle nh;

		ros::Publisher pub;
		//ros::Subscriber sub;
        ros::ServiceServer arm_cmd_srv;

		std_msgs::String cmd_msg;

		sensor_msgs::JointState joint_state_msg;
	public:
		EdwinInterface(ros::NodeHandle nh, const std::string&);
		ros::Time get_time();
		bool arm_cmd_cb(irl::arm_cmd::Request &req,irl::arm_cmd::Response &res);
		virtual void read(const ros::Time& time);
		virtual void write(const ros::Time& time);
};

enum {WRIST,HAND,ELBOW,SHOULDER,WAIST};
extern const std::string joints[];// = {"wrist","hand","elbow","shoulder","waist"};

#endif
