/*******************************************************************************
*                                                                              *
*   PrimeSense NITE 1.3 - Scene Analysis Sample                                *
*   Copyright (C) 2010 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
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

#include "SceneDrawer.h"

//ROS headers
#include <ros/ros.h>
#include <ros/package.h>  //for file paths

#include <stdlib.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

#include <sstream>

using std::string;

xn::Context g_Context;
xn::ScriptNode g_ScripeNode;
xn::DepthGenerator g_DepthGenerator;
xn::SceneAnalyzer g_SceneAnalyzer;
xn::Recorder* g_pRecorder;

// xml to initialize OpenNI

std::string path = ros::package::getPath("edwin");
std::string sample_file_path = "/src/Data/Sample-Scene.xml";
std::string x = path + sample_file_path;

const char* sample_file = x.c_str();

#define SAMPLE_XML_PATH sample_file


#include <GL/glut.h>
#include <GL/gl.h>

#include "signal_catch.h"

int counter = 1;


#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480
#define START_CAPTURE_CHECK_RC(rc, what)												\
	if (nRetVal != XN_STATUS_OK)														\
{																					\
	printf("Failed to %s: %s\n", what, xnGetStatusString(rc));				\
	StopCapture();															\
	return ;																	\
}

#define CHECK_ERRORS(rc, errors, what)		\
	if (rc == XN_STATUS_NO_NODE_PRESENT)	\
{										\
	XnChar strError[1024];				\
	errors.ToString(strError, 1024);	\
	printf("%s\n", strError);			\
	return (rc);						\
}

XnBool g_bDrawCoM = false;
XnBool g_bPause = false;
XnBool g_bStep = false;
XnBool g_bPrintFrameID = false;
XnBool g_bRecord = false;

XnBool g_bQuit = false;
void StopCapture()
{
	g_bRecord = false;
	if (g_pRecorder != NULL)
	{
		g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);
		g_pRecorder->Release();
		delete g_pRecorder;
	}
	g_pRecorder = NULL;
}

void CleanupExit()
{
	if (g_pRecorder)
		g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);
	StopCapture();

	exit (1);
}

void StartCapture()
{
	char recordFile[256] = {0};
	time_t rawtime;
	struct tm *timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	XnUInt32 size;
	xnOSStrFormat(recordFile, sizeof(recordFile)-1, &size,
		 "%d_%02d_%02d[%02d_%02d_%02d].oni",
		timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

	if (g_pRecorder != NULL)
	{
		StopCapture();
	}

	XnStatus nRetVal = XN_STATUS_OK;
	g_pRecorder = new xn::Recorder;

	g_Context.CreateAnyProductionTree(XN_NODE_TYPE_RECORDER, NULL, *g_pRecorder);
	START_CAPTURE_CHECK_RC(nRetVal, "Create recorder");

	nRetVal = g_pRecorder->SetDestination(XN_RECORD_MEDIUM_FILE, recordFile);
	START_CAPTURE_CHECK_RC(nRetVal, "set destination");
	nRetVal = g_pRecorder->AddNodeToRecording(g_DepthGenerator, XN_CODEC_16Z_EMB_TABLES);
	START_CAPTURE_CHECK_RC(nRetVal, "add node");
	g_bRecord = true;
}


void DrawProjectivePoints(XnPoint3D& ptIn, int width, double r, double g, double b)
{
	static XnFloat pt[3];

	pt[0] = ptIn.X;
	pt[1] = ptIn.Y;
	pt[2] = 0;

	glColor4f(r,
		g,
		b,
		1.0f);
	glPointSize(width);
	glVertexPointer(3, GL_FLOAT, 0, pt);
	glDrawArrays(GL_POINTS, 0, 1);



	glFlush();
  ros::spinOnce();

}
// this function is called each frame
void glutDisplay (void)
{

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);
	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);


	glDisable(GL_TEXTURE_2D);

	if (!g_bPause)
	{
		// Read next available data
		g_Context.WaitOneUpdateAll(g_DepthGenerator);
	}

	if (g_bStep)
	{
		g_bStep = false;
		g_bPause = true;
	}

	// Process the data
	//DRAW
	g_DepthGenerator.GetMetaData(depthMD);
	g_SceneAnalyzer.GetMetaData(sceneMD);

	DrawDepthMap(depthMD, sceneMD);
	if (g_bPrintFrameID)
	{
		DrawFrameID(depthMD.FrameID());
	}

	glutSwapBuffers();

}

void glutIdle (void)
{
	if (g_bQuit) {
		CleanupExit();
	}

	// Display the frame
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{

	switch (key)
	{
	case 27:
		CleanupExit();
	case'p':
		g_bPause = !g_bPause;
		break;
	case 'l':
		g_bPause = false;
		g_bStep = true;
		break;
	case 'c':
		g_bDrawCoM = !g_bDrawCoM;
		break;
	case 'f':
		g_bPrintFrameID = !g_bPrintFrameID;
		break;
	case 'k':
		if (g_pRecorder == NULL)
			StartCapture();
		else
			StopCapture();
		printf("Record turned %s\n", g_pRecorder ? "on" : "off");
		break;
	}
}
void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("PrimeSense Nite Scene Segmentation Viewer");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}




#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}





//-----------------------------------------------------------------------
//-------------------------------MAIN FUNCTION-----------------------------
//-----------------------------------------------------------------------

int main(int argc, char **argv)
{
	XnStatus rc = XN_STATUS_OK;
	xn::EnumerationErrors errors;

	//We initialize our ROS nodes and publishers here
	ros::init(argc, argv, "bodycontrol", ros::init_options::NoSigintHandler);
		ros::NodeHandle rosnode = ros::NodeHandle();

	  ros::Publisher pub_bodyX = rosnode.advertise<std_msgs::Int16>("body", 10);
	  std_msgs::Int16 msg_bodyX;



	rc = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_ScripeNode, &errors);
	CHECK_ERRORS(rc, errors, "InitFromXmlFile");
	CHECK_RC(rc, "InitFromXml");

	rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(rc, "Find depth generator");
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_SCENE, g_SceneAnalyzer);
	CHECK_RC(rc, "Find scene analyzer");

	rc = g_Context.StartGeneratingAll();
	CHECK_RC(rc, "StartGenerating");

	glInit(&argc, argv);
	glutMainLoop();


}
