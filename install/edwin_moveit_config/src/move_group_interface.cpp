/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Empty.h"

#include <iostream>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

typedef std::vector<double> d_vec;

void spawnObject(moveit::planning_interface::PlanningSceneInterface& p){

	std::vector<moveit_msgs::CollisionObject> v;

	// add table
	moveit_msgs::CollisionObject table;
	table.header.frame_id = "base_link";
	table.id = "table";
	shape_msgs::SolidPrimitive table_geom;
	table_geom.type = table_geom.BOX;
	table_geom.dimensions.push_back(2.032);
	table_geom.dimensions.push_back(0.6096);
	table_geom.dimensions.push_back(0.05);
	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 1;
	table_pose.position.x = 0.55;
	table_pose.position.y = 0;
	table_pose.position.z = -0.05;
	table.primitives.push_back(table_geom);
	table.primitive_poses.push_back(table_pose);
	table.operation = table.ADD;
	v.push_back(table);

	// add kinect
	moveit_msgs::CollisionObject kinect;
	kinect.header.frame_id = "base_link";
	kinect.id = "kinect";

	shape_msgs::SolidPrimitive kinect_geom;
	kinect_geom.type = kinect_geom.BOX;
	kinect_geom.dimensions.push_back(0.28);
	kinect_geom.dimensions.push_back(0.08);
	kinect_geom.dimensions.push_back(0.08);

	geometry_msgs::Pose kinect_pose;
	kinect_pose.orientation.w = 1;
	kinect_pose.position.x = 0.635;
	kinect_pose.position.y = 0.30;
	kinect_pose.position.z = 0.04;
	kinect.primitives.push_back(kinect_geom);
	kinect.primitive_poses.push_back(kinect_pose);
	kinect.operation = kinect.ADD;
	v.push_back(kinect);

	// add "OBJECT"
	//moveit_msgs::CollisionObject object;
	//object.header.frame_id = "odom";
	//object.id = "object";

	//shape_msgs::SolidPrimitive primitive;
	//primitive.type = primitive.CYLINDER;
	//primitive.dimensions.push_back(0.2);
	//primitive.dimensions.push_back(0.04);

	//geometry_msgs::Pose pose;
	//pose.orientation.w = 1;
	//pose.position.x = 0.3; // 30 cm forwards from frame
	//pose.position.y = 0.0;
	//pose.position.z = primitive.dimensions[0]/2 + 0.0;

	//object.primitives.push_back(primitive);
	//object.primitive_poses.push_back(pose);

	//// add object to scene
	//object.operation = object.ADD;
	//v.push_back(object);
	//p.addCollisionObjects(v);
	p.applyCollisionObjects(v);
	return;
}

struct solutionSort{
	const d_vec& seed_pose;
	solutionSort(const d_vec& seed_pose):seed_pose(seed_pose){
	}
	bool operator()(const d_vec& v_1, const d_vec& v_2){
		int n = v_1.size();
		float d_1 = 0.0;
		float d_2 = 0.0;
		//ASSERT n == 5
		for(int i=0; i<n; ++i){
			d_1 += fabs(v_1[i] - seed_pose[i]);
			d_2 += fabs(v_2[i] - seed_pose[i]);
		}	
		return d_1 < d_2;
	}	
};

class EdwinMoveGroupInterface{
	private:
		const ros::NodeHandle& nh;
		ros::Subscriber sub, j_sub, g_sub;

		ros::Publisher marker_pub;
		visualization_msgs::Marker marker_msg;

		tf::TransformListener tf_listener;

		moveit::planning_interface::MoveGroup group;
		const moveit::core::JointModelGroup& j_group;
		geometry_msgs::Pose target_pose;

	public:
		EdwinMoveGroupInterface(ros::NodeHandle& nh);
		void obj_cb(const geometry_msgs::PointStampedConstPtr& msg);
		void move_cb(const std_msgs::EmptyConstPtr& msg);
		void joint_cb(const sensor_msgs::JointStateConstPtr& msg);

		bool moveToPose(const geometry_msgs::Pose& target_pose);
};


bool EdwinMoveGroupInterface::moveToPose(const geometry_msgs::Pose& target_pose){
	group.setStartStateToCurrentState();
	moveit::planning_interface::MoveGroup::Plan my_plan;
	const kinematics::KinematicsBaseConstPtr& solver_ptr = j_group.getSolverInstance();

	std::vector<geometry_msgs::Pose> target_pose_v;
	target_pose_v.push_back(target_pose);

	d_vec seed_pose;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), seed_pose);
	std::vector<d_vec> solutions;

	kinematics::KinematicsResult result;
	kinematics::KinematicsQueryOptions options;

	options.return_approximate_solution = true;
	options.discretization_method = kinematics::DiscretizationMethods::NO_DISCRETIZATION; // TODO : play with this

	if(solver_ptr->getPositionIK(target_pose_v,seed_pose,solutions,result,options)){
		ROS_INFO("ALTERNATIVE IK SOLUTION CANDIDATES FOUND! ");
		std::cout << std::setprecision(4);

		std::vector<std::string> link_names;
		link_names.push_back(group.getEndEffectorLink());

		// sorting with least difference as beginning, most difference as end
		// "prefer" a solution closer to the current state
		std::sort(solutions.begin(), solutions.end(), solutionSort(seed_pose));

		for(std::vector<d_vec>::const_iterator it = solutions.begin(); it != solutions.end(); ++it){
			const d_vec& sol = (*it);
			float pitch = fabs(sol[1] + sol[2] + sol[3]);
			ROS_INFO("PITCH : %f\n", pitch);

			if(fabs(fabs(pitch) - M_PI) > 1e-1){
				// Sometimes it searches for things pointing upwards, which doesn't work.
				// Therefore, we're looking for an ik solution pointing DOWNWARDS!
				continue;
			}

			group.setJointValueTarget(sol);
			bool success = group.plan(my_plan);
			if(success){
				//std::cout << sol[1] << ',' << sol[2] << ',' << sol[3] << std::endl;
				ROS_INFO("Random Pose Goal SUCCESS");
				group.move();
				return true;
			}
		}
	}
	return false;
}

void EdwinMoveGroupInterface::joint_cb(const sensor_msgs::JointStateConstPtr& msg){
	const std::vector<double>& p = msg->position;
	group.setStartStateToCurrentState();
	group.setJointValueTarget(p);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);
	ROS_INFO("Joint Request :  %s",success?"SUCCESS":"FAILED");
	if(success){
		group.move();
	}
}

EdwinMoveGroupInterface::EdwinMoveGroupInterface(ros::NodeHandle& nh):
	nh(nh),
	tf_listener(ros::Duration(5)),
	group("arm_group"),
	j_group(*group.getRobotModel()->getJointModelGroup("arm_group"))
{
	// **** ROS SUB/PUB *** //
	sub = nh.subscribe("obj_point", 1, &EdwinMoveGroupInterface::obj_cb, this);
	j_sub = nh.subscribe("joint_cmd", 1, &EdwinMoveGroupInterface::joint_cb, this);
	g_sub = nh.subscribe("move", 1, &EdwinMoveGroupInterface::move_cb, this);
	marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10, true);

	// **** CONFIGURE GROUP **** //
	group.setNumPlanningAttempts(8); // attempt three times
	group.setGoalPositionTolerance(0.01); // 1cm tolerance
	group.setGoalOrientationTolerance(0.034); // 2 deg. tolerance
	group.setGoalJointTolerance(0.034);

	group.setSupportSurfaceName("table");
	group.setStartStateToCurrentState();
	group.setWorkspace(-2,-2,0,2,2,2);
	group.setPlannerId("RRTConnectkConfigDefault");

	// **** SETUP ENVIRONMENT **** //
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
	spawnObject(planning_scene_interface);

	// **** DISPLAY RELEVANT INFO **** //
	ROS_INFO("Reference frame (Plan): %s", group.getPlanningFrame().c_str());
	ROS_INFO("Reference frame (End Effector): %s", group.getEndEffectorLink().c_str());
	geometry_msgs::Pose p = group.getCurrentPose().pose;
	ROS_INFO("Current Pose: %f %f %f | %f %f %f %f\n", p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);

	// **** FORMAT MARKER MESSAGE **** //
	marker_msg.header.frame_id = "/odom";
	marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 0.05;
	marker_msg.type = marker_msg.SPHERE;
	marker_msg.id = 0;
	marker_msg.pose.orientation.w = 1.0; // believe auto filled to 0
	marker_msg.color.a = 1;
	marker_msg.color.r = 1;
	marker_msg.color.g = 0;
	marker_msg.color.b = 1;
}

void EdwinMoveGroupInterface::obj_cb(const geometry_msgs::PointStampedConstPtr& msg){
	group.setStartStateToCurrentState();
	geometry_msgs::PointStamped target_point;

	int n = 5;
	for(int i=0; i < n; ++i){
		try{
			//ROS_INFO("Frame ID : %s", msg->header.frame_id.c_str());
			tf_listener.waitForTransform("base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
			tf_listener.transformPoint("base_link", *msg, target_point);
			//tf_listener.transformPoint("base_link", ros::Time(0), *msg, "odom", target_point);
			marker_msg.pose.position = target_point.point;
			marker_pub.publish(marker_msg);

//void Transformer::lookupTransform( const std::string &target_frame, const std::string &source_frame, const ros::Time &time, tf::StampedTransform &transform ) const
//void Transformer::lookupTransform( const std::string &target_frame, const ros::Time &target_time, const std::string &source_frame, const ros::Time &source_time, const std::string &fixed_frame, tf::StampedTransform &transform ) const


			moveit::planning_interface::MoveGroup::Plan my_plan;

			target_pose.position = target_point.point;
			target_pose.position.x += 0.036; // adjust x so that the gripper would be aligned with the object
			target_pose.position.z += 0.2;  // raise a little bit to have the end of the jaw set at the object

			tf::Quaternion q1 = tf::Quaternion(tf::Vector3(0,1,0),M_PI/2);
			tf::Quaternion q2 = tf::Quaternion(tf::Vector3(0,0,1),0);
			tf::Quaternion q = q1*q2;
			tf::quaternionTFToMsg(q, target_pose.orientation);
			break;
		}catch(tf::TransformException e){
			std::string what = e.what();
			ROS_INFO("Invalid Object Point :\n%s", e.what());
			//target_point.header.frame_id = "base_link";
			//target_point.header.stamp = ros::Time::now();
			//target_point.point = msg->point;
		}catch(tf::LookupException e){
			std::string what = e.what();
			ROS_INFO("Invalid Object Point :\n%s", e.what());
		}catch(...){
			ROS_INFO("Invalid Object Point. Sadness!");
		}
	}
}

void EdwinMoveGroupInterface::move_cb(const std_msgs::EmptyConstPtr&){
	geometry_msgs::Pose p = target_pose;

	// approach object from above
	p.position.z += 0.2;
	bool s1, s2;
	s1 = moveToPose(p);
	if(s1){
		p.position.z -= 0.2; // now actually get to object
		s2 = moveToPose(p);
	}
	// TODO : call gripper here
	ROS_INFO("Grip : %s / %s", (s1?"SUCCESS":"FAIL"), (s2?"SUCCESS":"FAIL"));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edwin_move_group_interface");
	ros::NodeHandle nh;  

	EdwinMoveGroupInterface edwin(nh);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	// from here listen to callbacks
	ros::Rate r = ros::Rate(10.0);
	ros::Time last_pose_update = ros::Time::now();
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}

	ros::shutdown();  
	return 0;
}
