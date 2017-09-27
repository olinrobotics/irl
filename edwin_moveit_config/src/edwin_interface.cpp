#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "utils.h"
#include "edwin_interface.h"
#include "Mutex.h"

#include <algorithm>
#include <string>

const std::string joints[] = {"waist","shoulder","elbow","hand","wrist"};
Mutex mtx;

void split(std::string& s, const std::string& delim, std::vector<std::string>& res){
    res.clear();
    size_t pos=0;
    std::string token;
    while ((pos = s.find(delim)) != std::string::npos) {
        token = s.substr(0, pos);
        res.push_back(token);
        s.erase(0, pos + delim.length());
    }
    res.push_back(s);
}
// string manip.
std::vector<std::string> split(
        std::string s,
        const std::string& delim){
    std::vector<std::string> res;
    split(s,delim,res);
    return res;
}



EdwinInterface::EdwinInterface(ros::NodeHandle nh, const std::string& dev):st(dev), nh(nh){

	// initialize arm
    ROS_INFO("Initializing ST Arm!");
	st.initialize();
	st.start();
	st.set_speed(10000);
	st.home();
    ROS_INFO("Initialization Complete.");

	for(int i=0; i<N_JOINTS; ++i){
		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle(joints[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(state_handle);
		// connect and register the joint position interface
		hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joints[i]), &cmd[i]);
		jnt_pos_interface.registerHandle(pos_handle);
	}
	registerInterface(&jnt_pos_interface);

	pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10, false);
	//sub = nh.subscribe("arm_cmd", 10, &EdwinInterface::arm_cmd_cb, this);
    arm_cmd_srv = nh.advertiseService("arm_cmd", &EdwinInterface::arm_cmd_cb, this);

	//publish joint states
	joint_state_msg.header.frame_id = "base_link";

	// urdf joint names
	for(int i=0; i<N_JOINTS; ++i){
		joint_state_msg.name.push_back(joints[i]);
		joint_state_msg.position.push_back(0);
		joint_state_msg.velocity.push_back(0);
		joint_state_msg.effort.push_back(0);
	}
}

ros::Time EdwinInterface::get_time(){
	return ros::Time::now();
}

bool EdwinInterface::arm_cmd_cb(
        irl::arm_cmd::Request& req,
        irl::arm_cmd::Response& res
        ){

    auto raw_cmds = split(req.cmd.data(), "data: ");
    if(raw_cmds.size() <= 0){
        return false;
    }

    raw_cmds = split(raw_cmds.back(), ":: ");

    std::string cmd, param;
    cmd = raw_cmds[0];
    if(raw_cmds.size() > 1){
        param = raw_cmds[1];
    }

    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
    std::transform(param.begin(), param.end(), param.begin(), ::tolower);

    if(cmd == "de_energize"){
        //
    }else if(cmd == "energize"){
        //
    }else if(cmd == "where"){
        auto loc = st.where();
    }else if(cmd == "create_route"){

    }else if(cmd == "calibrate"){
        st.calibrate();
    }else if(cmd == "home"){
        st.home();
    }

    //// ##### MUTEX #####
	//mtx.lock();
	//st.write(msg->data);
	//mtx.unlock();
	//// #################

	// TODO : implement backwards-compatible arm-cmd
	//joint_state_msg = *msg;
	//for(int i=0; i<N_JOINTS; ++i){
	//	pos[i] = joint_state_msg.position[i];
	//}
}

void cvtJ(std::vector<double>& j){
	// convert to degrees
	j[0] *= 90./B_RATIO;
	j[1] *= 90./S_RATIO;
	j[2] *= 90./E_RATIO;
	j[3] *= 90./W_RATIO;
	j[4] *= 90./T_RATIO;
	j[4] -= j[3];

	for(std::vector<double>::iterator it = j.begin(); it!=j.end();++it){
		double& l = *it;
		l = d2r(l);
	}
}

void cvtJ_i(std::vector<double>& j){
	//invert the conversion
	for(std::vector<double>::iterator it = j.begin(); it!=j.end();++it){
		double& l = *it;
		l = r2d(l);
	}
	j[4] += j[3];

	j[0] = int(j[0] * B_RATIO / 90.);
	j[1] = int(j[1] * S_RATIO / 90.);
	j[2] = int(j[2] * E_RATIO / 90.);
	j[3] = int(j[3] * W_RATIO / 90.);
	j[4] = int(j[4] * T_RATIO / 90.);
}

void EdwinInterface::read(const ros::Time& time){
	//alias with reference
	std::vector<double>& loc = joint_state_msg.position;
	
	// ##### MUTEX #####
	mtx.lock();	
	st.where(loc);
	mtx.unlock();

	if(loc.size() != 5){
		std::cerr << "WARNING :: INVALID LOCATION READ !!! " << std::endl;
		std::cerr << loc.size() << std::endl;
		return;
	}
	// #################

	//reformat based on scale and direction
	cvtJ(loc);

	// fill data
	for(int i=0; i<N_JOINTS;++i){
		pos[i] = loc[i];
		//std::cout << pos[i] << ',';
	}
	//std::cout << std::endl;

	//publish joint states
	joint_state_msg.header.stamp = ros::Time::now();
	pub.publish(joint_state_msg);
}

void EdwinInterface::write(const ros::Time& time){
	//st.move ... joints
	// info to st-r17, i.e. desired joint states
	std::vector<double> cmd_pos(N_JOINTS);
	for(int i=0; i<N_JOINTS; ++i){
		cmd_pos[i] = cmd[i];
	}

	cvtJ_i(cmd_pos); // invert conversion


	// ##### MUTEX #####
	mtx.lock();
	st.move(cmd_pos);
	//for(int i=0; i<N_JOINTS; ++i){
	//	// DEBUGGING:
	//	// std::cout << (i==0?"":", ") << cmd[i];
	//	
	//	float dp = cmd[i] - pos[i]; // target-position
	//	dp = dp>0?dp:-dp; // abs

	//	if(dp > d2r(1)){ // more than 1 degrees different
	//		// TODO : apply scaling factors?
	//		st.move(joints[i], cmd_pos[i]);
	//	}
	//}
	mtx.unlock();
	// #################
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v){
	os << '[';
	for(typename std::vector<T>::const_iterator it=v.begin();it!=v.end();++it){
		os << (*it) << ',';
	}
	os << ']';
}

int main(int argc, char* argv[]){
	ros::init(argc,argv,"edwin_hardware");
	std::vector<double> j;

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	std::string dev;
	nh_priv.param("/dev", dev, std::string("/dev/ttyUSB0")); 
	ROS_INFO("Get Param %s", ros::param::get("/dev", dev)?"SUCCEEDED":"FAILED");
	ROS_INFO("initialized with device %s", dev.c_str());

	EdwinInterface edwin(nh, dev);
	controller_manager::ControllerManager cm(&edwin, nh);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Time then = edwin.get_time();
	ros::Rate r = ros::Rate(10.0); // 10 Hz
	int cnt = 0;
	while(ros::ok()){
		ros::Time now = edwin.get_time();
		ros::Duration period = ros::Duration(now-then);
		edwin.read(now);
		cm.update(now, period);
		edwin.write(now);
		++cnt;
		r.sleep();
	}
	return 0;
}
