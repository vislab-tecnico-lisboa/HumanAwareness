#include "../include/tracker/cameraModel.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../include/tracker/detectionProcess.hpp"
#include <ros/ros.h>
#include "sensor_msgs/CameraInfo.h"
#include "../include/tracker/filtersAndUtilities.hpp"
#include <fstream>



#define PERSON_HEIGHT 1.8

using namespace cv;
using namespace std;

cameraModel::cameraModel(string configFile, string cameraStr)
{
  //Getting camera intrinsics from camera info topic

  ROS_INFO("Getting camera parameters");

  sensor_msgs::CameraInfoConstPtr l_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraStr, ros::Duration(30));

  ROS_ERROR_STREAM(cameraStr);

  //Check if timed out. If there is no topic, then load default parameters from file!

  if (l_camera_info == NULL)
  {
    //Failed. Just load default file
      FileStorage fs(configFile, FileStorage::READ);
      if(!fs.isOpened())
        {
          cout << "Couldn't open camera config file." << endl;
          exit(-4);
        }

       fs["camera_matrix"] >> K_;
       ROS_ERROR("No cameraInfoTopic found. Loading default parameters. RESULTS WILL BE BAD!");
  }
  else
  {
  K_ = Mat::eye(3,3,CV_64F);
  K_.at<double>(0,0) = l_camera_info->K.at(0);
  K_.at<double>(1,1) = l_camera_info->K.at(4);
  K_.at<double>(0,2) = l_camera_info->K.at(2);
  K_.at<double>(1,2) = l_camera_info->K.at(5);
  }

  invert(K_, invertedK);
  invertedK.convertTo(invertedK, CV_32FC1);


  //Transforms points from camera frame to the base_footprint frame
  //This is an approximation, considering that the camera is pointing to the front,
  //the optical axis is parallel to the floor. Only used if I don't have TF's

  float pose_array[] = {0, 0, 1, 0, -1, 0, 0, 0.1, 0, -1, 0, 0.95, 0, 0, 0, 1};
  pose = Mat(4, 4, CV_32FC1, pose_array).clone();

}


/*
*  This is the "best" way to calculate the points on the base frame, since it uses real time transformations
*  between the camera and world frames.
*  For now it should only work on simulation, because the real Vizzy only uses YARP on its upper body...
*
*  But is actually performing poorer that the other methods, by 2 meters!
*  I'm probably doing something wrong...
*  UPDATE: I wasn't... It was a bug on Gazebo. It's working awesomely well :)
*
*/
vector<Point3d> cameraModel::calculatePointsOnWorldFrame(Mat imagePoints, Mat worldLinkToCamera, vector<cv::Rect_<int> >rects)
{

  //Transform the points to homogeneous coordinates

  vector<Point3d> basePoints;

  if(!imagePoints.empty())
  {
  Mat transposedPoints;

  transpose(imagePoints, transposedPoints);
  Mat ones = Mat::ones(1, transposedPoints.size().width, CV_32FC1);


  transposedPoints.convertTo(transposedPoints, CV_32FC1);
  transposedPoints.push_back(ones);

  Mat &homogeneousPoints = transposedPoints;

  //First normalize the points
  // K^(-1)*x_cam = [R|t]*p

  //If pz = 0 then, we get a homography wich we can invert

  Mat homography_tmp(4, 3, 6);

  Mat homography(3,3, 6);

  worldLinkToCamera.col(0).copyTo(homography_tmp.col(0));
  worldLinkToCamera.col(1).copyTo(homography_tmp.col(1));
  worldLinkToCamera.col(3).copyTo(homography_tmp.col(2));

  homography = homography_tmp(Range(0, 3), Range(0, 3));

  homography.convertTo(homography, CV_32FC1);

  Mat invertedHomography;
  invert(homography, invertedHomography);


  Mat normalizedPoints = invertedK*homogeneousPoints;

  //Finally we get the points on the base frame in homogeneous coordinates
  // p = H^-1 * (K^-1 * x_cam)

  Mat homogeneousP = invertedHomography*normalizedPoints;

  //Now we just get the x, y from the homogeneous coordinates and set z to 0
  /*
            [p1x p2x p3x ... pnx]
  p_tilde=  [p1y p2y p3y ... pny]
            [l_1 l_2 l_3 ... l_n]
  */

  //x = pix/l_i
  //y = piy/l_i


   for(int i = 0; i< homogeneousP.size().width; i++)
     {
       Point3d point;
       point.x = homogeneousP.at<float>(0, i)/homogeneousP.at<float>(2, i);
       point.y = homogeneousP.at<float>(1, i)/homogeneousP.at<float>(2, i);

       Point2d bbCenter = getCenter(rects.at(i));

       point.z = getZ(bbCenter, Point2d(point.x, point.y), worldLinkToCamera, this);

       ROS_ERROR_STREAM("altura: " << 2*point.z);

       basePoints.push_back(point);
     }

  }
   return basePoints;

}

//This method is used if we dont have access to Vizzy's upper body tfs.
//Performing suprisingly good.

vector<Point3d> cameraModel::calculatePointsOnWorldFrameWithoutHomography(vector<cv::Rect_<int> >* rects, Mat baseLinkToWorld)
{
  Mat imagePointsWithDepth(4, rects->size(), CV_32FC1);

  vector<cv::Rect_<int> >::iterator it;
  int i = 0;
  for(it = rects->begin(); it != rects->end(); it++)
  {
    Point2d center;
    center = getCenter(*it);

    //z = f*H/h_image
    float z;

    z = K_.at<float>(0,0)*PERSON_HEIGHT/(*it).height;

    imagePointsWithDepth.at<float>(2, i) = z;

    imagePointsWithDepth.at<float>(0, i) = z*(center.x*invertedK.at<float>(0,0)+invertedK.at<float>(0,2));
    imagePointsWithDepth.at<float>(1, i) = z*(center.y*invertedK.at<float>(1,1)+invertedK.at<float>(1,2));

    imagePointsWithDepth.at<float>(3, i) = 1;

    i++;
  }

  //Get the points in the base link frame
  Mat pointsInBaseFrame;
  pointsInBaseFrame = pose*imagePointsWithDepth;

  //And finally in the world frame
  Mat pointsInWorldFrame;
  baseLinkToWorld.convertTo(baseLinkToWorld, CV_32FC1);
  pointsInWorldFrame = baseLinkToWorld*pointsInBaseFrame;

  vector<Point3d> worldPoints;
  for(int i=0; i < pointsInWorldFrame.size().width; i++)
  {
    Point3d point;
    point.x = pointsInWorldFrame.at<float>(0, i);
    point.y = pointsInWorldFrame.at<float>(1, i);
    point.z = 0;

    worldPoints.push_back(point);
  }

  return worldPoints;

}

Mat cameraModel::getK()
{
  return K_;
}


Mat cameraModel::getDistCoefs()
{
  return distCoefs_;
}


Mat cameraModel::getProjectionMat()
{
  return projectionMat_;
}
