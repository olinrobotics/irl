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

EdwinInterface::EdwinInterface(ros::NodeHandle nh, const std::string& dev):st(dev), nh(nh){

	// initialize arm
    ROS_INFO("Initializing ST Arm!");
	st.initialize();
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
    registerInterface(&jnt_state_interface);
	registerInterface(&jnt_pos_interface);

	//pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10, false);
    arm_cmd_srv = nh.advertiseService("/arm_cmd", &EdwinInterface::arm_cmd_cb, this);

	//publish joint states
	joint_state_msg.header.frame_id = "base_link";

	// urdf joint names
	for(int i=0; i<N_JOINTS; ++i){
		joint_state_msg.name.push_back(joints[i]);
		joint_state_msg.position.push_back(0);
	}
    // unknown joints
    joint_state_msg.velocity.clear();
    joint_state_msg.effort.clear();
}
EdwinInterface::~EdwinInterface(){

}
ros::Time EdwinInterface::get_time(){
	return ros::Time::now();
}

bool EdwinInterface::arm_cmd_cb(
        irl::arm_cmd::Request& req,
        irl::arm_cmd::Response& res
        ){

    ROS_INFO("Received Service Request : %s", req.cmd.c_str()); 
    // Format : 
    // data: cmd:: param
    
    // parse cmd ...
    auto raw_cmds = split(req.cmd, "data: ");
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
    ROS_INFO("Received CMD : [%s]; PARAM : [%s]", cmd.c_str(), param.c_str());
    // parse complete
     
    // obtain ST Arm Control
	mtx.lock();
    // act on cmd ...
    if(cmd == "de_energize"){
        st.de_energize();
    }else if(cmd == "energize"){
        st.energize();
    }else if(cmd == "where"){
        ROS_INFO("Command [WHERE] Deprecated; subscribe to /joint_states topic instead.");
    }else if(cmd == "create_route"){
        ROS_INFO("Command [CREATE_ROUTE] Not Supported.");
    }else if(cmd == "calibrate"){
        st.calibrate();
    }else if(cmd == "home"){
        st.home();
    }else if(cmd == "get_speed"){
        auto speed = st.get_speed();
        std::string msg = ("SPEED : " + std::to_string(speed));
        res.response = msg;
    }else if(cmd == "set_speed"){
        auto speed = std::stoi(param);
        st.set_speed(speed);
        res.response = "SUCCESS";
    }else if(cmd == "get_accel"){
        auto accel = st.get_accel();
        std::string msg = ("ACCEL : " + std::to_string(accel));
        res.response = msg;
    }else if(cmd == "set_accel"){
        auto accel = std::stoi(param);
        st.set_accel(accel);
        res.response = "SUCCESS";
    }else if(cmd == "run_route"){

    }else if(cmd == "move_to"){

    }else if(cmd == "rotate_wrist"){

    }else if(cmd == "rotate_wrist_rel"){

    }
    mtx.unlock();

	// TODO : implement backwards-compatible arm-cmd
	//joint_state_msg = *msg;
	//for(int i=0; i<N_JOINTS; ++i){
	//	pos[i] = joint_state_msg.position[i];
	//}
    return true;
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

	// fill data -- let controller manager know
	for(int i=0; i<N_JOINTS;++i){
		pos[i] = loc[i];
	}

	//publish joint states
	joint_state_msg.header.stamp = ros::Time::now();
	//pub.publish(joint_state_msg);
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

	while(ros::ok()){
		ros::Time now = edwin.get_time();
		ros::Duration period = ros::Duration(now-then);
        then = now;

		edwin.read(now);
		cm.update(now, period);
		edwin.write(now);

		r.sleep();
	}
	return 0;
}
