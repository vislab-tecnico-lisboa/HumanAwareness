//General purpose includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <stack>
#include <string>


//ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Our detector
#include "../include/pedestrianDetector.hpp"


using namespace std;
//using namespace cv;

static const string OPENCV_WINDOW = "Detector";




class PedDetector
{
  private:
  //ROS NodeHandle, and image_transport objects
  ros::NodeHandle nh;
  image_transport::ImageTransport *it;
  image_transport::ImageTransport *it2;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  
  //Our detector
  pedestrianDetector *detector;
  
  //Callback to process the images
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat image(cv_ptr->image);  
  
  vector<cv::Rect_<int> >* rects = detector->runDetector(image);
  
  vector<cv::Rect_<int> >::iterator it;
    for(it = rects->begin(); it != rects->end(); it++){
		rectangle(image, *it, Scalar_<int>(255,0,0));
	}   

  cv::waitKey(30);
  
  
//Publish the resulting image for now. Later I might publish only the detections for another node to process
    
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  image_pub.publish(out_msg);


//  cv::imshow(OPENCV_WINDOW, image);
//  cv::waitKey(30);
  
  }
 
  public:
  PedDetector(string conf)
  {
	//Initialization
	detector = new pedestrianDetector(conf);
	it = new image_transport::ImageTransport(nh);
	
	//Advertise
	image_pub = it->advertise("/pedestriandetector/image", 1);
	//Subscribe to vizzy's left camera
    image_sub = it->subscribe("/vizzy/l_camera/image_raw", 1, &PedDetector::imageCb, this);
    
  }
  
  ~PedDetector() 
  {
    delete detector;
  }
  
  
};


int main(int argc, char** argv)
{
  
  
  ros::init(argc, argv, "pedestrianDetector");
//  cv::namedWindow(OPENCV_WINDOW);
  
  
  PedDetector detector("configuration.xml");
  
  ros::spin();
  return 0;	
}
