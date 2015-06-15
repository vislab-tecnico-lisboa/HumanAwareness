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
#include <visualization_msgs/Marker.h>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Our detector
#include "../include/pedestrianDetector.hpp"

//Other includes
#include "../include/controller.hpp"
#include "../include/detectionProcess.hpp"
#include "../include/cameraModel.hpp"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "opencv2/core/eigen.hpp"
#include "geometry_msgs/PoseStamped.h"

using namespace std;
using namespace cv;

static const string OPENCV_WINDOW = "Detector";




class PedDetector
{
    tf::TransformListener listener;

private:
    //ROS NodeHandle, and image_transport objects
    ros::NodeHandle nh;
    ros::Publisher control_pub;
    ros::Publisher marker_pub;
    image_transport::ImageTransport *it;
    image_transport::ImageTransport *it2;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    visualization_msgs::Marker marker;

    //Our detector
    pedestrianDetector *detector;

    //Camera model
    cameraModel *cameramodel;

    //Callback to process the images
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        tf::StampedTransform transform;
        try
        {
//            listener.waitForTransform("/l_camera_vision_link", "/odom", ros::Time(0), ros::Duration(10.0) );

//            listener.lookupTransform("/l_camera_vision_link", "/odom",ros::Time(0), transform);

            listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0) );

            listener.lookupTransform("/odom", "/base_footprint",ros::Time(0), transform);


        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }


        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(transform, eigen_transform);


        // convert image from Eigen to openCV
        cv::Mat transform_opencv;
        cv::eigen2cv(eigen_transform.matrix(), transform_opencv);

        cv::Mat odomToBaseLinkTransform;

        invert(transform_opencv, odomToBaseLinkTransform);

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
            rectangle(image, *it, Scalar_<int>(0,255,0), 3);
            circle(image, getFeet(*it), 3, Scalar_<int>(0, 0, 255), 2);
            stringstream ss;
            ss << getPersonDistance(*it);
            putText(image, ss.str(), getFeet(*it), FONT_HERSHEY_SIMPLEX, 1, Scalar_<int>(0, 0, 255), 1);
        }

        //Publish the resulting image for now. Later I might publish only the detections for another node to process

        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub.publish(out_msg);

        //Calculate the position from camera intrinsics and extrinsics

        Mat feetImagePoints;

        for(it = rects->begin(); it!= rects-> end(); it++)
        {
            Mat feetMat(getFeet(*it));

            transpose(feetMat, feetMat);

            feetImagePoints.push_back(feetMat);
        }

        if(rects->size() > 0)
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
           segwayController::moveBase(coordsInBaseFrame[0], odomToBaseLinkTransform, control_pub);
        }

    }

public:
    PedDetector(string conf, string cameraConfig)
    {
        //Initialization
        cameramodel = new cameraModel(cameraConfig);
        detector = new pedestrianDetector(conf);
        it = new image_transport::ImageTransport(nh);


        //Rviz marker - It will show people detections on Rviz
        marker_pub = nh.advertise<visualization_msgs::Marker>("person", 1);
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

        marker.lifetime = ros::Duration();



        //Base control message
        control_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);



        //Advertise
        image_pub = it->advertise("image_out", 1);


        //Subscribe to vizzy's left camera
        //Change this later
        image_sub = it->subscribe("/vizzy/l_camera/image_raw", 1, &PedDetector::imageCb, this);

    }

    ~PedDetector()
    {
        delete detector;
        delete cameramodel;
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
