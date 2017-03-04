
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sstream>

#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include<string>
#include <time.h>

class FaceTracker{
  public:
    bool detect;
    time_t started_tracking;

    int main(int argc, char **argv){
      FaceTracker self;
      ros::init(argc, argv, "face_tracking", ros::init_options::AnonymousName);
      ros::NodeHandle n  = ros::NodeHandle();
      ros::Publisher pub = n.advertise<std_msgs::String>("/face_location", 10);
      ros::Subscriber sub_control = n.subscribe("all_control", 10, cmd_callback);

      CvBridge bridge = CvBridge();
      ros::Subscriber sub_bridge = n.subscribe("usb_cam/image_raw", img_callback);

      detect = true;
      started_tracking = time(0);
      std::string PACKAGE_PATH = ros::package::getPath("edwin");
      CascadeClassifier face_cascade;
      face_cascade.load(PACKAGE_PATH + "/params/haarcascade_frontalface_default.xml");

      int ideal_x = 640/2;
      int ideal_y = 480/2;
    }

    void img_callback(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr frame;
      try
      {
        frame = cv_bridge::toCvCopy(msg, sensor_msgs:image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what())
      }

    }

    void cmd_callback(const std_msgs::String::ConstPtr& msg){
      if (msg.find("ft stop") != string::npos){
        detect = false;
      }
      else if (msg.find("ft go") != string::npos){
        started_tracking = time(0);
        this.detect = true;
      }
    }

    void face_tracking(){
      std::vector<rect> faces;
      cv::Mat gray;
      cv::cvtColor(frame, gray, CV_BGR2GRAY) //Convert the frame to gray.
      face_cascade.detectMultiScale(gray, faces, 1.3, 5);
      float largest_face [4] = {0, 0, 0, 0};
      // Set Region of Interest
      cv::Rect roi_b;
      cv::Rect roi_c;

      size_t ic  = 0; // ic is index of current element
      int ac = 0; // ac is area of current element

      size_t ib = 0; //ib is index of biggest element
      int ab = 0; //ab is area of a biggest element

      roi_b.x = faces[ib].x;
      roi_b.y = faces[ib].y;
      roi_b.width = (faces[ib].width);
      roi_b.height = (faces[ib].height);

      ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element


      for (ic = 0; ic < faces.size(); ic++){
        //roi_c.x = faces[ic].x;
        //roi_c.y = faces[ic].y;
        roi_c.width = (faces[ic].width);
        roi_c.height = (faces[ic].height);

        ac = roi_c.width * roi_c.height; //get area of current element.

        if (ac > ab)
        {
            ib = ic;
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
        }
      }
      // http://stackoverflow.com/questions/20757147/detect-faces-in-image

      //std::string locx = static_cast<ostringstream*>(&(ostringstream()<<roi_b.x))->str();
      //std::string locy = static_cast<ostringstream*>(&(ostringstream()<<roi_b.y))->str();
      std::string locx = std::to_string(roi_b.x);
      std::string locy = std::to_string(roi_b.y);

      pub.publish(locx + ":" + locy);
    }

    void run(){
      ros::Rate r(10);
      while (ros::ok()){
        if (self.detect){
          face_tracking();
        }
        r.sleep();
      }
    }
};
