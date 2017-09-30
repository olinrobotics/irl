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

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
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


moveit::planning_interface::MoveGroup* group_ptr;
const moveit::core::JointModelGroup* j_group_ptr;

ros::Publisher marker_pub;
visualization_msgs::Marker marker_msg;
tf::TransformListener* listener;

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

bool moveToPose(const geometry_msgs::Pose& target_pose){
	if(group_ptr == NULL || j_group_ptr == NULL)
		return false;
	moveit::planning_interface::MoveGroup& group = *group_ptr;
	group.setStartStateToCurrentState();
	moveit::planning_interface::MoveGroup::Plan my_plan;

	const moveit::core::JointModelGroup& j_group = *j_group_ptr;
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

geometry_msgs::Pose target_pose;

void obj_cb(const geometry_msgs::PointStampedConstPtr& msg){
	if(group_ptr == NULL && j_group_ptr == NULL)
		return;

	moveit::planning_interface::MoveGroup& group = *group_ptr;
	group.setStartStateToCurrentState();

	geometry_msgs::PointStamped target_point;
	try{
		listener->transformPoint("base_link", *msg, target_point);
	}catch(...){
		target_point.header.frame_id = "base_link";
		target_point.header.stamp = ros::Time::now();
		target_point.point = msg->point;
	}

	marker_msg.pose.position = target_point.point;
	marker_pub.publish(marker_msg);

	moveit::planning_interface::MoveGroup::Plan my_plan;

	//group.setPositionTarget(
	//		target_point.point.x,
	//		target_point.point.y,
	//		target_point.point.z,
	//		"link_5");
	target_pose.position = target_point.point;
	target_pose.position.x += 0.036; // adjust x so that the gripper would be aligned with the object
	target_pose.position.z += 0.16;  // raise a little bit to have the end of the jaw set at the object

	tf::Quaternion q1 = tf::Quaternion(tf::Vector3(0,1,0),M_PI/2);
	tf::Quaternion q2 = tf::Quaternion(tf::Vector3(0,0,1),0);
	tf::Quaternion q = q1*q2;
	tf::quaternionTFToMsg(q, target_pose.orientation);
}

void grip_cb(const std_msgs::Empty& msg){
	geometry_msgs::Pose p = target_pose;

	// approach object from above
	p.position.z += 0.2;
	bool s1, s2;
	s1 = moveToPose(p);
	if(s1){
		p.position.z -= 0.2;
		s2 = moveToPose(p);
	}

	ROS_INFO("Grip : %s / %s", (s1?"SUCCESS":"FAIL"), (s2?"SUCCESS":"FAIL"));
}

void joint_cb(const std_msgs::Float64MultiArrayConstPtr& msg){
	moveit::planning_interface::MoveGroup& group = *group_ptr;
	group.setStartStateToCurrentState();
	group.setJointValueTarget(msg->data);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);
	ROS_INFO("Joint Request :  %s",success?"SUCCESS":"FAILED");
	if(success){
		group.move();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edwin_move_group_interface");
	ros::NodeHandle nh;  
	ros::Subscriber sub = nh.subscribe("obj_point", 1, obj_cb);
	ros::Subscriber sub_j = nh.subscribe("joint_positions", 1, joint_cb);
	ros::Subscriber sub_g = nh.subscribe("grip", 1, grip_cb);

	marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10, true);

	listener = new tf::TransformListener();

	ros::Publisher valid_marker_pub = nh.advertise<visualization_msgs::Marker>("valid_marker", 10, true);
	visualization_msgs::Marker valid_marker_msg;
	ros::Publisher valid_marker_pub_2 = nh.advertise<visualization_msgs::Marker>("valid_marker_2", 10, true);
	visualization_msgs::Marker valid_marker_msg_2;

	marker_msg.header.frame_id = "/odom";
	marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 0.05;
	marker_msg.type = marker_msg.SPHERE;
	marker_msg.id = 0;
	marker_msg.pose.orientation.w = 1.0; // believe auto filled to 0
	marker_msg.color.a = 1;
	marker_msg.color.r = 1;
	marker_msg.color.g = 0;
	marker_msg.color.b = 1;

	ros::AsyncSpinner spinner(1);

	spinner.start();

	/* This sleep is ONLY to allow Rviz to come up */
	// BEGIN_TUTORIAL
	// 
	// Setup
	// ^^^^^
	// 
	// The :move_group_interface:`MoveGroup` class can be easily 
	// setup using just the name
	// of the group you would like to control and plan for.

	moveit::planning_interface::MoveGroup group("arm_group");

	try{

		j_group_ptr = group.getRobotModel()->getJointModelGroup("arm_group");
		ROS_ASSERT(j_group_ptr != NULL);
	}catch(...){
		ROS_INFO("FAILURE!!");
	}

	group.setNumPlanningAttempts(8); // attempt three times
	group_ptr = &group; 

	group.setGoalPositionTolerance(0.03); // 3cm tolerance
	group.setGoalOrientationTolerance(0.034); // 2 deg. tolerance
	group.setGoalJointTolerance(0.034);

	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
	spawnObject(planning_scene_interface);

	group.setSupportSurfaceName("table");
	group.setStartStateToCurrentState();
	group.setWorkspace(-2,-2,0,2,2,2);
	group.setPlannerId("RRTConnectkConfigDefault");
	//group.setStartStateToCurrentState();

	// (Optional) Create a publisher for visualizing plans in Rviz.
	//ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	//moveit_msgs::DisplayTrajectory display_trajectory;

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame (Plan): %s", group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame (End Effector): %s", group.getEndEffectorLink().c_str());

	geometry_msgs::Pose p = group.getCurrentPose().pose;
	ROS_INFO("Current Pose: %f %f %f | %f %f %f %f\n", p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);

	//
	// Visualizing plans
	// ^^^^^^^^^^^^^^^^^
	// Now that we have a plan we can visualize it in Rviz.  This is not
	// necessary because the group.plan() call we made above did this
	// automatically.  But explicitly publishing plans is useful in cases that we
	// want to visualize a previously created plan.
	//if (1)
	//{
	//  ROS_INFO("Visualizing plan 1 (again)");    
	//  display_trajectory.trajectory_start = my_plan.start_state_;
	//  display_trajectory.trajectory.push_back(my_plan.trajectory_);
	//  display_publisher.publish(display_trajectory);
	//  /* Sleep to give Rviz time to visualize the plan. */
	//  sleep(5.0);
	//}

	//#define GO_HOME
#ifdef GO_HOME
	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	// Now, let's modify one of the joints, plan to the new joint
	// space goal and visualize the plan.
	for(std::vector<double>::iterator it = group_variable_values.begin(); it != group_variable_values.end(); ++it){
		*it = 0.0; // set home
	}

	group.setJointValueTarget(group_variable_values);
	success = group.plan(my_plan);
	ROS_INFO("Plan 2 (joint space goal) %s",success?"":"FAILED");
	if(success)
		group.move();
#endif

	//#define VALIDATE_POSITIONS
#ifdef VALIDATE_POSITIONS
	group.setStartStateToCurrentState();
	moveit::planning_interface::MoveGroup::Plan valid_plan;
	geometry_msgs::Point& p = marker_msg.pose.position;

	valid_marker_msg = marker_msg;
	valid_marker_msg.type = 8;

	valid_marker_msg_2 = valid_marker_msg;

	// validate marker positions
	for(float x=-0.8; x<=0.8; x+=0.1){
		for(float y=-1.0; y<=1.0; y+=0.1){
			for(float z=0.0; z<=1.5;z+=0.1){
				ROS_INFO("x:%f,y:%f,z:%f\n",x,y,z);
				p.x = x;
				p.y = y;
				p.z = z;
				group.setPositionTarget(x,y,z,"link_5");
				bool success = group.plan(valid_plan);
				std_msgs::ColorRGBA col;
				col.a = 1.0;
				if(success){
					col.g=col.b=1.0;
					col.r=0.0;
					valid_marker_msg_2.colors.push_back(col);
					valid_marker_msg_2.points.push_back(p);
					valid_marker_pub_2.publish(valid_marker_msg_2);
				}else{
					col.g=col.b=0.0;
					col.r=1.0;
				}
				valid_marker_msg.colors.push_back(col);
				valid_marker_msg.points.push_back(p);
				valid_marker_pub.publish(valid_marker_msg);
			}
		}
	}
#endif

	// from here listen to callbacks
	ros::Rate r = ros::Rate(10.0);
	ros::Time last_pose_update = ros::Time::now();
	while(ros::ok()){
		//ros::Time now = ros::Time::now();
		//if( (now - last_pose_update).toSec() > 1.0){
		//	last_pose_update = now;
		//	geometry_msgs::PoseStamped p = group.getRandomPose("link_5");
		//	group.setPoseTarget(p, "link_5");
		//	if (group.plan(my_plan))
		//		group.move();
		//}
		ros::spinOnce();
		r.sleep();
	}

	//sleep(3.0); // sleep for 10 sec.

	//moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	//moveit_simple_grasps::SimpleGrasps simple_grasps;

	//std::vector<moveit_msgs::Grasp> grasps;
	//moveit_msgs::Grasp g;
	//group.setPlanningTime(10.0);
	//group.setSupportSurfaceName("table");

	//moveit::planning_interface::MoveItErrorCode err = group.planGraspsAndPick(obj);
	//std::cerr << err << std::endl;

	//group.pick(obj.id, grasps);
	//sleep(10.0);

	/* Sleep to give Rviz time to visualize the plan. */
	// Planning with Path Constraints
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Path constraints can easily be specified for a link on the robot.
	// Let's specify a path constraint and a pose goal for our group.
	// First define the path constraint.
	//moveit_msgs::OrientationConstraint ocm;  
	//ocm.link_name = "r_wrist_roll_link";  
	//ocm.header.frame_id = "base_link";
	//ocm.orientation.w = 1.0;
	//ocm.absolute_x_axis_tolerance = 0.1;
	//ocm.absolute_y_axis_tolerance = 0.1;
	//ocm.absolute_z_axis_tolerance = 0.1;
	//ocm.weight = 1.0;
	//
	//// Now, set it as the path constraint for the group.
	//moveit_msgs::Constraints test_constraints;
	//test_constraints.orientation_constraints.push_back(ocm);  
	//group.setPathConstraints(test_constraints);

	//// We will reuse the old goal that we had and plan to it.
	//// Note that this will only work if the current state already 
	//// satisfies the path constraints. So, we need to set the start
	//// state to a new pose. 
	//robot_state::RobotState start_state(*group.getCurrentState());
	//geometry_msgs::Pose start_pose2;
	//start_pose2.orientation.w = 1.0;
	//start_pose2.position.x = 0.55;
	//start_pose2.position.y = -0.05;
	//start_pose2.position.z = 0.8;
	//const robot_state::JointModelGroup *joint_model_group =
	//                start_state.getJointModelGroup(group.getName());
	//start_state.setFromIK(joint_model_group, start_pose2);
	//group.setStartState(start_state);
	//
	//// Now we will plan to the earlier pose target from the new 
	//// start state that we have just created.
	//group.setPoseTarget(target_pose1);
	//success = group.plan(my_plan);

	//ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
	///* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);

	//// When done with the path constraint be sure to clear it.
	//group.clearPathConstraints();

	// Cartesian Paths
	// ^^^^^^^^^^^^^^^
	// You can plan a cartesian path directly by specifying a list of waypoints 
	// for the end-effector to go through. Note that we are starting 
	// from the new start state above.  The initial pose (start state) does not
	// need to be added to the waypoint list.
	//std::vector<geometry_msgs::Pose> waypoints;

	//geometry_msgs::Pose target_pose3 = start_pose2;
	//target_pose3.position.x += 0.2;
	//target_pose3.position.z += 0.2;
	//waypoints.push_back(target_pose3);  // up and out

	//target_pose3.position.y -= 0.2;
	//waypoints.push_back(target_pose3);  // left

	//target_pose3.position.z -= 0.2;
	//target_pose3.position.y += 0.2;
	//target_pose3.position.x -= 0.2;
	//waypoints.push_back(target_pose3);  // down and right (back to start)

	//// We want the cartesian path to be interpolated at a resolution of 1 cm
	//// which is why we will specify 0.01 as the max step in cartesian
	//// translation.  We will specify the jump threshold as 0.0, effectively
	//// disabling it.
	//moveit_msgs::RobotTrajectory trajectory;
	//double fraction = group.computeCartesianPath(waypoints,
	//                                             0.01,  // eef_step
	//                                             0.0,   // jump_threshold
	//                                             trajectory);

	//ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
	//      fraction * 100.0);    
	///* Sleep to give Rviz time to visualize the plan. */
	//sleep(15.0);


	// Adding/Removing Objects and Attaching/Detaching Objects
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// First, we will define the collision object message.
	//moveit_msgs::CollisionObject collision_object;
	//collision_object.header.frame_id = group.getPlanningFrame();

	///* The id of the object is used to identify it. */
	//collision_object.id = "box1";

	///* Define a box to add to the world. */
	//shape_msgs::SolidPrimitive primitive;
	//primitive.type = primitive.BOX;
	//primitive.dimensions.resize(3);
	//primitive.dimensions[0] = 0.4;
	//primitive.dimensions[1] = 0.1;
	//primitive.dimensions[2] = 0.4;

	///* A pose for the box (specified relative to frame_id) */
	//geometry_msgs::Pose box_pose;
	//box_pose.orientation.w = 1.0;
	//box_pose.position.x =  0.6;
	//box_pose.position.y = -0.4;
	//box_pose.position.z =  1.2;

	//collision_object.primitives.push_back(primitive);
	//collision_object.primitive_poses.push_back(box_pose);
	//collision_object.operation = collision_object.ADD;

	//std::vector<moveit_msgs::CollisionObject> collision_objects;  
	//collision_objects.push_back(collision_object);  

	//// Now, let's add the collision object into the world
	//ROS_INFO("Add an object into the world");  
	//planning_scene_interface.addCollisionObjects(collision_objects);
	//
	///* Sleep so we have time to see the object in RViz */
	//sleep(2.0);

	//// Planning with collision detection can be slow.  Lets set the planning time
	//// to be sure the planner has enough time to plan around the box.  10 seconds
	//// should be plenty.
	//group.setPlanningTime(10.0);


	//// Now when we plan a trajectory it will avoid the obstacle
	//group.setStartState(*group.getCurrentState());
	//group.setPoseTarget(target_pose1);
	//success = group.plan(my_plan);

	//ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
	//  success?"":"FAILED");
	///* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);
	//

	//// Now, let's attach the collision object to the robot.
	//ROS_INFO("Attach the object to the robot");  
	//group.attachObject(collision_object.id);  
	///* Sleep to give Rviz time to show the object attached (different color). */
	//sleep(4.0);


	//// Now, let's detach the collision object from the robot.
	//ROS_INFO("Detach the object from the robot");  
	//group.detachObject(collision_object.id);  
	///* Sleep to give Rviz time to show the object detached. */
	//sleep(4.0);


	//// Now, let's remove the collision object from the world.
	//ROS_INFO("Remove the object from the world");  
	//std::vector<std::string> object_ids;
	//object_ids.push_back(collision_object.id);  
	//planning_scene_interface.removeCollisionObjects(object_ids);
	///* Sleep to give Rviz time to show the object is no longer there. */
	//sleep(4.0);

	ros::shutdown();  
	delete listener;
	return 0;
}
