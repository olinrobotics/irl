/*******************************************************************************
*                                                                              *
*   PrimeSense NITE 1.3 - Scene Analysis Sample                                *
*   Copyright (C) 2010 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef XNV_POINT_DRAWER_H_
#define XNV_POINT_DRAWER_H_

#include <XnCppWrapper.h>

//ROS headers
#include <ros/ros.h>
#include <ros/package.h>  //for file paths

#include <stdlib.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

#include <sstream>

void DrawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd, ros::Publisher pub_body, std_msgs::String msg_body);
void DrawFrameID(XnUInt32 nFrameID);

#endif
