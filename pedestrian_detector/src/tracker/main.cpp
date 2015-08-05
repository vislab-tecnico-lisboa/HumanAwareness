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
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>


//Our includes
#include "../include/tracker/detectionProcess.hpp"
#include "../include/tracker/cameraModel.hpp"
#include "../include/tracker/personMotionModel.hpp"

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
    ros::Subscriber image_sub;
    bool personNotChosenFlag;
    Point3d targetCoords;
    PersonList personList;

    int targetId;

    //Marker stuff
    interactive_markers::InteractiveMarkerServer *marker_server;
    visualization_msgs::InteractiveMarker int_marker;

    //Message
    ros::Publisher position_publisher;


  public:
    //Process the clicks on markers!
    void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {

        //If we click the marker for the first time it will choose that detection to track
        if(personNotChosenFlag)
          {
            stringstream ss;
            ss << feedback->marker_name;

            string trash;

            ss >> trash >> targetId;

            cout << "Target! Id = " << targetId << endl;

            personNotChosenFlag = false;
          }
        else
        //The second time we click on the marker we stop following that person. (Only click after some delay are counted...)
          {
            personNotChosenFlag = true;
          }

      ROS_INFO_STREAM( feedback->marker_name << " is now at "
          << feedback->pose.position.x << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z );




    }

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
//        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0) );
//        listener.lookupTransform("/map", "/base_footprint",ros::Time(0), transform);

            //The following transforms are used if we want to calculate the position using a homography. It's kinda unstable
            //to do it that way
          listener.waitForTransform("/map", "/l_camera_vision_link", ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform("/map", "/l_camera_vision_link",ros::Time(0), transform);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      Eigen::Affine3d eigen_transform;
      tf::transformTFToEigen(transform, eigen_transform);



      // convert matrix from Eigen to openCV
      cv::Mat transform_opencv;
      cv::eigen2cv(eigen_transform.matrix(), transform_opencv);

      cv::Mat odomToBaseLinkTransform;

      invert(transform_opencv, odomToBaseLinkTransform);

      cout << odomToBaseLinkTransform << endl;

      exit(0);

      //Calculate the position from camera intrinsics and extrinsics

      Mat feetImagePoints;

      for(vector<cv::Rect_<int> >::iterator it = rects.begin(); it!= rects.end(); it++)
      {
      Mat feetMat(getFeet(*it));

      transpose(feetMat, feetMat);

      feetImagePoints.push_back(feetMat);
      }


      vector<cv::Point3d> coordsInBaseFrame;

//      coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrameWithoutHomography(&rects,transform_opencv);

      coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrame(feetImagePoints, odomToBaseLinkTransform);


      personList.associateData(coordsInBaseFrame, rects);

      vector<PersonModel> list = personList.getValidTrackerPosition();

      //Send the rects correctly ordered and identified back to the detector so that we can view it on the image

       /*
        *  TODO!
        *
        */

      //If we haven't chosen a person to follow, show all detections with a green marker on Rviz
      //that have median different than -1000 on either coordinate

      if(personNotChosenFlag)
      {

        marker_server->clear();

        for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
        {
            stringstream description, name;
            name << "person " << (*it).id;
            description << "Detection " << (*it).id;
            int_marker.name = name.str();
            int_marker.description = description.str();

            Point3d position = (*it).medianFilter();

            int_marker.controls.at(0).markers.at(0).color.r = 0;
            int_marker.controls.at(0).markers.at(0).color.g = 1;
            int_marker.controls.at(0).markers.at(0).color.b = 0;

            int_marker.pose.position.x = position.x;
            int_marker.pose.position.y = position.y;

            cout << "Sending to marker server: " << "(" << position.x << "," << position.y << ")" << endl;

            marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));
        }

        marker_server->applyChanges();
      }
      else
      {

        marker_server->clear();
        cout << "Person " << targetId << " chosen" << endl;
        for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
        {

            Point3d position = (*it).medianFilter();

            cout << "(*it).id =" << (*it).id << endl;

            if((*it).id == targetId)
            {

              it->lockedOnce = true;

              int_marker.controls.at(0).markers.at(0).color.r = 1;
              int_marker.controls.at(0).markers.at(0).color.g = 0;
              int_marker.controls.at(0).markers.at(0).color.b = 0;

              int_marker.pose.position.x = position.x;
              int_marker.pose.position.y = position.y;


              stringstream description, name;
              name << "person " << (*it).id;
              description << "Objective: Detection " << (*it).id;

              int_marker.name = name.str();
              int_marker.description = description.str();

              marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));


              //Now we send out the position
              geometry_msgs::Point final_position;

              final_position.x = position.x;
              final_position.y = position.y;
              final_position.z = 0;

              position_publisher.publish(final_position);
            }
            else
            {
              int_marker.controls.at(0).markers.at(0).color.r = 0;
              int_marker.controls.at(0).markers.at(0).color.g = 1;
              int_marker.controls.at(0).markers.at(0).color.b = 0;

              Point3d position = (*it).medianFilter();

              int_marker.pose.position.x = position.x;
              int_marker.pose.position.y = position.y;

              stringstream description, name;

              name << "person " << (*it).id;
              description << "Detection " << (*it).id;

              int_marker.name = name.str();
              int_marker.description = description.str();

              marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));

            }


        }

        marker_server->applyChanges();

      }
    }
    Tracker(string cameraConfig)
    {

      personNotChosenFlag = true;

      cameramodel = new cameraModel(cameraConfig);
      image_sub = n.subscribe("detections", 1, &Tracker::trackingCallback, this);

      //Prepare the marker
      marker_server = new interactive_markers::InteractiveMarkerServer("tracker");

      visualization_msgs::Marker person_marker;

      person_marker.type = visualization_msgs::Marker::CYLINDER;
      person_marker.scale.x = 0.3;  //0.3
      person_marker.scale.y = 0.3; //0.3
      person_marker.scale.z = 1;  //1
      person_marker.color.r = 0;
      person_marker.color.g = 1;
      person_marker.color.b = 0;
      person_marker.color.a = 1.0;
      person_marker.pose.position.z = 0.5;

      visualization_msgs::InteractiveMarkerControl click_me;
      click_me.always_visible = true;
      click_me.markers.push_back(person_marker);
      click_me.name = "click";
      click_me.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

      int_marker.header.frame_id = "/map";
      int_marker.header.stamp=ros::Time::now();

      int_marker.controls.push_back(click_me);

      position_publisher = n.advertise<geometry_msgs::Point>("person_position", 1);


    }

    ~Tracker()
    {
      delete cameramodel;
      delete marker_server;
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
