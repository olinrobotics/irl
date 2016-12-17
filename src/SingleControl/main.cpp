/*******************************************************************************
*                                                                              *
*   PrimeSense NITE 1.3 - Single Control Sample                                *
*   Copyright (C) 2010 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

//-----------------------------------------------------------------------------
// Headers
//-----------------------------------------------------------------------------
// General headers
#include <stdio.h>
// OpenNI headers
#include <XnOpenNI.h>
// NITE headers
#include <XnVSessionManager.h>
#include "XnVMultiProcessFlowClient.h"
#include <XnVWaveDetector.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

//ROS headers
#include <ros/ros.h>
#include <ros/package.h>  //for file paths

#include <stdlib.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"


#include <edwin/pointerpos.h>
#include <edwin/edwin_gestures.h>
#include <sstream>


using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

// xml to initialize OpenNI

std::string path = ros::package::getPath("edwin");
std::string sample_file_path = "/src/Data/Sample-Tracking.xml";
std::string x = path + sample_file_path;
std::string sample_file_local_path = "Sample-Tracking.xml";
std::string y = path + sample_file_local_path;

const char* sample_file = x.c_str();
const char* sample_file_local = y.c_str();

#define SAMPLE_XML_FILE sample_file
#define SAMPLE_XML_FILE_LOCAL sample_file_local

//variables that will be used in sending out ROS messages
int xpos;
int ypos;
int zpos;
int num;
bool wave;
bool sess_start;
bool sess_end;
bool operation = true;


//-----------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------

// Callback for when the focus is in progress
void XN_CALLBACK_TYPE SessionProgress(const XnChar* strFocus, const XnPoint3D& ptFocusPoint, XnFloat fProgress, void* UserCxt)
{
	printf("Session progress (%6.2f,%6.2f,%6.2f) - %6.2f [%s]\n", ptFocusPoint.X, ptFocusPoint.Y, ptFocusPoint.Z, fProgress,  strFocus);
}
// callback for session start
void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& ptFocusPoint, void* UserCxt)
{
	printf("Session started. Please wave (%6.2f,%6.2f,%6.2f)...\n", ptFocusPoint.X, ptFocusPoint.Y, ptFocusPoint.Z);
	sess_start = true;
}
// Callback for session end
void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
	printf("Session ended. Please perform focus gesture to start session\n");
	sess_end = true;
}
// Callback for wave detection
void XN_CALLBACK_TYPE OnWaveCB(void* cxt)
{
	printf("Wave!\n");
	wave = true;
}
// callback for a new position of any hand
void XN_CALLBACK_TYPE OnPointUpdate(const XnVHandPointContext* pContext, void* cxt)
{
	printf("%d: (%f,%f,%f) [%f]\n", pContext->nID, pContext->ptPosition.X, pContext->ptPosition.Y, pContext->ptPosition.Z, pContext->fTime);
	xpos = pContext->ptPosition.X;
	ypos = pContext->ptPosition.Y;
	zpos = pContext->ptPosition.Z;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

XnBool fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> tokens;
    while (getline(ss, item, delim)) {
        tokens.push_back(item);
    }
    return tokens;
}
void toggle_callback(const std_msgs::String::ConstPtr& msg)
{
	printf("i got something");
	std::string test = msg->data.c_str();

	std::vector<std::string> x = split(test, ';');
	for (unsigned i=0; i < x.size(); i++)
	{
    std::vector<std::string> y = split(x[i], ':');
		if (y[0] == "singlecontrol")
		{
			if (y[1] == "stop")
			{
				printf("STOPPING");
				operation = false;
			}
			else
			{
				printf("RUNNING");
				operation = true;
			}

		}


	}

}



// this sample can run either as a regular sample, or as a client for multi-process (remote mode)
int main(int argc, char** argv)
{

	xn::Context context;
	xn::ScriptNode scriptNode;
	XnVSessionGenerator* pSessionGenerator;
	XnBool bRemoting = FALSE;


	if (argc > 1)
	{
		// remote mode
		context.Init();
		printf("Running in 'Remoting' mode (Section name: %s)\n", argv[1]);
		bRemoting = TRUE;

		// Create multi-process client
		pSessionGenerator = new XnVMultiProcessFlowClient(argv[1]);

		XnStatus rc = ((XnVMultiProcessFlowClient*)pSessionGenerator)->Initialize();
		if (rc != XN_STATUS_OK)
		{
			printf("Initialize failed: %s\n", xnGetStatusString(rc));
			delete pSessionGenerator;
			return 1;
		}
	}
	else
	{
		// Local mode
		// Create context
		const char *fn = NULL;
		if      (fileExists(SAMPLE_XML_FILE)) fn = SAMPLE_XML_FILE;
		else if (fileExists(SAMPLE_XML_FILE_LOCAL)) fn = SAMPLE_XML_FILE_LOCAL;
		else {
			printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_FILE, SAMPLE_XML_FILE_LOCAL);
			return XN_STATUS_ERROR;
		}
		XnStatus rc = context.InitFromXmlFile(fn, scriptNode);
		if (rc != XN_STATUS_OK)
		{
			printf("Couldn't initialize: %s\n", xnGetStatusString(rc));
			return 1;
		}

		// Create the Session Manager
		pSessionGenerator = new XnVSessionManager();
		rc = ((XnVSessionManager*)pSessionGenerator)->Initialize(&context, "Click", "RaiseHand");
		if (rc != XN_STATUS_OK)
		{
			printf("Session Manager couldn't initialize: %s\n", xnGetStatusString(rc));
			delete pSessionGenerator;
			return 1;
		}

		// Initialization done. Start generating
		context.StartGeneratingAll();
	}

	// Register session callbacks
	pSessionGenerator->RegisterSession(NULL, &SessionStart, &SessionEnd, &SessionProgress);

	// init & register wave control
	XnVWaveDetector wc;
	wc.RegisterWave(NULL, OnWaveCB);
	wc.RegisterPointUpdate(NULL, OnPointUpdate);
	pSessionGenerator->AddListener(&wc);

	printf("Please perform focus gesture to start session\n");
	printf("Hit any key to exit\n");

	//We initialize our ROS nodes and publishers here
	ros::init(argc, argv, "pointcontrol", ros::init_options::NoSigintHandler);
  	ros::NodeHandle rosnode = ros::NodeHandle();


  	ros::Publisher pub_waving = rosnode.advertise<std_msgs::Int16>("wave_at_me", 10);
  	std_msgs::Int16 msg_waving;

		ros::Subscriber toggle = rosnode.subscribe<std_msgs::String>("all_control", 10, &toggle_callback);



	// Main loop
	while (!xnOSWasKeyboardHit())
	{
		if(operation)
		{
			if (bRemoting)
			{
				((XnVMultiProcessFlowClient*)pSessionGenerator)->ReadState();
			}
			else
			{
				context.WaitAnyUpdateAll();
				((XnVSessionManager*)pSessionGenerator)->Update(&context);

				//fill out the custom message fields with the placeholders we've previously defined
	/*			msg.positionx = xpos;
				msg.positiony = ypos;
				msg.positionz = zpos;
				pub.publish(msg);
				msg_gestures.wave = wave;
				msg_gestures.hello = sess_start;
				msg_gestures.goodbye = sess_end;
				pub_gestures.publish(msg_gestures);
	*/
				if(wave)
				{
					num = 1;
				}
				else
				{
					num = 0;
				}

				msg_waving.data = num;
				pub_waving.publish(msg_waving);

				//set gesture booleans back to false for re-initialization again
				wave = false;
				sess_start = false;
				sess_end = false;

		}


		}
		ros::spinOnce();

	}

	delete pSessionGenerator;

	return 0;
}
