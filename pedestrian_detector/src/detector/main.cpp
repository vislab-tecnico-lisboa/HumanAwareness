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
//    MoveBaseClient *ac = new MoveBaseClient("move_base", true);
//    ros::Publisher marker_pub;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
//    visualization_msgs::Marker marker;

    //Our detector
    pedestrianDetector *detector;

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


        vector<cv::Rect_<int> >* rects = detector->runDetector(image);


        pedestrian_detector::DetectionList detectionList;


        //Print rectangles on the image and add them to the detection list
        vector<cv::Rect_<int> >::iterator it;
        for(it = rects->begin(); it != rects->end(); it++){
            rectangle(image, *it, Scalar_<int>(0,255,0), 3);
            pedestrian_detector::BoundingBox bb;
            bb.x = (*it).x;
            bb.y = (*it).y;
            bb.width = (*it).width;
            bb.height = (*it).height;
            detectionList.bbVector.push_back(bb);
        }

        //Publish the resulting image for now. Later I might publish only the detections for another node to process

        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub.publish(out_msg);

        //Publish the detections (I will also publish the features associated to each detection)
        detectionPublisher.publish(detectionList);



        //Calculate the position from camera intrinsics and extrinsics

/*        if(rects->size() > 0)
        {
           vector<cv::Point3d> coordsInBaseFrame;

//          coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrame(feetImagePoints, transform_opencv);

           coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrameWithoutHomography(rects,transform_opencv);

           cout << coordsInBaseFrame << endl;

           marker.pose.position.x = coordsInBaseFrame[0].x;
           marker.pose.position.y = coordsInBaseFrame[0].y;
           marker.pose.position.z = coordsInBaseFrame[0].z;

           marker_pub.publish(marker);


           //Send the control command
           //segwayController::moveBase(coordsInBaseFrame[0], odomToBaseLinkTransform, *ac);
        }*/

    }

public:
    PedDetector(string conf, string cameraConfig)
    {
        //Initialization
        detector = new pedestrianDetector(conf);
        it = new image_transport::ImageTransport(nh);


        //Rviz marker - It will show people detections on Rviz
 /*       marker_pub = nh.advertise<visualization_msgs::Marker>("person", 1);
        uint32_t shape = visualization_msgs::Marker::CYLINDER;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "person";
        marker.id = 0;

        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration();*/


        //Advertise
        image_pub = it->advertise("image_out", 1);
        detectionPublisher = nh.advertise<pedestrian_detector::DetectionList>("detections", 1);

        //Subscribe to vizzy's left camera
        //Change this later
        image_sub = it->subscribe("/vizzy/l_camera/image_raw", 1, &PedDetector::imageCb, this);

    }

    ~PedDetector()
    {
        delete detector;
        delete it;
    }


};


int main(int argc, char** argv)
{


    ros::init(argc, argv, "pedestrianDetector");

    //Get package path. This way we dont need to worry about running the node in the folder of the configuration files
    stringstream ss, ss2;
    ss << ros::package::getPath("pedestrian_detector");
    ss2 << ros::package::getPath("pedestrian_detector");

    ss << "/configuration.xml";
    ss2 << "/camera_model/config.yaml";

    PedDetector detector(ss.str(), ss2.str());

    ros::spin();
    return 0;
}
