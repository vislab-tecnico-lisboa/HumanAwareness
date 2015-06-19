/*******************************************
*
*   Person Tracker
*
*******************************************/

//ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "opencv2/core/eigen.hpp"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>

//Our includes
#include "../include/tracker/detectionProcess.hpp"
#include "../include/tracker/cameraModel.hpp"

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

//Custom messages
#include <pedestrian_detector/DetectionList.h>
#include <pedestrian_detector/BoundingBox.h>

//Other includes
#include <sstream>
#include <stack>
#include <iostream>

using namespace std;

class Tracker{

  private:
    cameraModel *cameramodel;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::NodeHandle n;
    ros::Subscriber sub;

  public:
    void trackingCallback(const pedestrian_detector::DetectionList::ConstPtr &detection)
    {

      //Get rects from message
      vector<cv::Rect_<int> > rects;

      for(pedestrian_detector::DetectionList::_bbVector_type::const_iterator it = detection->bbVector.begin(); it != detection ->bbVector.end(); it++)
      {
        int x, y, width, height;
        x = (*it).x;
        y = (*it).y;
        width = (*it).width;
        height = (*it).height;
        cv::Rect_<int> detect(x, y, width, height);
        rects.push_back(detect);
      }

      //Get transforms
      tf::StampedTransform transform;
      try
      {
        listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/odom", "/base_footprint",ros::Time(0), transform);
      }
      catch(tf::TransformException ex)
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

      //Calculate the position from camera intrinsics and extrinsics

      Mat feetImagePoints;

      for(vector<cv::Rect_<int> >::iterator it = rects.begin(); it!= rects.end(); it++)
      {
        Mat feetMat(getFeet(*it));

        transpose(feetMat, feetMat);

        feetImagePoints.push_back(feetMat);
      }

      vector<cv::Point3d> coordsInBaseFrame;
      coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrameWithoutHomography(&rects,transform_opencv);

      cout << coordsInBaseFrame;

      /*
      *  Now we apply some filtering on the detections
      */

    }

    Tracker(string cameraConfig)
    {
      cameramodel = new cameraModel(cameraConfig);
      sub = n.subscribe("detections", 1, &Tracker::trackingCallback, this);
    }

    ~Tracker()
    {
      delete cameramodel;
    }

};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracker");

  stringstream ss;

  ss << ros::package::getPath("pedestrian_detector");
  ss << "/camera_model/config.yaml";

  Tracker tracker(ss.str());

  ros::spin();

  return 0;
}
