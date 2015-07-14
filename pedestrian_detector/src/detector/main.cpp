/*******************************************
*
*   Pedestrian Detector
*
*******************************************/

//General purpose includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <stack>
#include <string>
#include <sstream>
//#include <thread>

//ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Our detector
#include "../include/detector/pedestrianDetector.hpp"

//Our custom messages
#include <pedestrian_detector/DetectionList.h>
#include <pedestrian_detector/BoundingBox.h>
#include <pedestrian_detector/Features.h>


using namespace std;
using namespace cv;

class PedDetector
{

private:
    //ROS NodeHandle, and image_transport objects
    ros::NodeHandle nh;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    //Our detectors
    pedestrianDetector *person_detector;
    pedestrianDetector *heads_detector;

    //Detections publisher
    ros::Publisher detectionPublisher;

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


        person_detector->runDetector(image);
        heads_detector->runDetector(image);




        pedestrian_detector::DetectionList detectionList;


        //Print rectangles on the image and add them to the detection list
        //Print detection numbers so that we can initialize the tracker using Rviz
        //And send a marker with each detection number to Rviz
        vector<cv::Rect_<int> >::iterator it;

        for(it = person_detector->boundingBoxes->begin(); it != person_detector->boundingBoxes->end(); it++){
            rectangle(image, *it, Scalar_<int>(0,255,0), 3);

            pedestrian_detector::BoundingBox bb;
            bb.x = (*it).x;
            bb.y = (*it).y;
            bb.width = (*it).width;
            bb.height = (*it).height;
            detectionList.bbVector.push_back(bb);
        }

        for(it = heads_detector->boundingBoxes->begin(); it != heads_detector->boundingBoxes->end(); it++)
        {
            rectangle(image, *it, Scalar_<int>(0,0,255), 3);
        }

        //Publish the resulting image for now. Later I might publish only the detections for another node to process

        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub.publish(out_msg);

        //Publish the detections (I will also publish the features associated to each detection)
        detectionPublisher.publish(detectionList);



    }

public:
    PedDetector(string conf_pedestrians, string conf_heads)
    {
        //Initialization
        person_detector = new pedestrianDetector(conf_pedestrians);
        heads_detector = new pedestrianDetector(conf_heads);
        it = new image_transport::ImageTransport(nh);

        //Advertise
        image_pub = it->advertise("image_out", 1);
        detectionPublisher = nh.advertise<pedestrian_detector::DetectionList>("detections", 1);

        //Subscribe to vizzy's left camera
        //Change this later
        image_sub = it->subscribe("/vizzy/l_camera/image_raw", 1, &PedDetector::imageCb, this);

    }

    ~PedDetector()
    {
        delete person_detector;
        delete heads_detector;
        delete it;
    }


};


int main(int argc, char** argv)
{


    ros::init(argc, argv, "pedestrianDetector");

    //Get package path. This way we dont need to worry about running the node in the folder of the configuration files
    stringstream ss;
    ss << ros::package::getPath("pedestrian_detector");
    ss << "/configuration.xml";

    stringstream ss2;
    ss2 << ros::package::getPath("pedestrian_detector");
    ss2 << "/configurationheadandshoulders.xml";

    PedDetector detector(ss.str(), ss2.str());

    ros::spin();
    return 0;
}
