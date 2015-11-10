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

//Stuff for results from simulation...
#include <nav_msgs/Odometry.h>
#include <fstream>

//Gaze control
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_robot_msgs/GazeAction.h>

using namespace std;


//Stuff for results from simulation...
cv::Point2d person1;
cv::Point2d person2;
int frame = 1;
//ofstream results;


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
    Point3d lastFixationPoint;



    actionlib::SimpleActionClient<move_robot_msgs::GazeAction> ac;
    ros::Time imageStamp;


    int targetId;

    //Marker stuff
    interactive_markers::InteractiveMarkerServer *marker_server;
    visualization_msgs::InteractiveMarker int_marker;

    //Message
    ros::Publisher position_publisher;


    //Things to get results...
//    ros::Subscriber person1Topic;
//    ros::Subscriber person2Topic;

    std::string cameraStr;
    std::string mapTopic;
    ros::NodeHandle nPriv;

  public:

    //Stuff to get results!
/*    void person1PosCallback(const nav_msgs::OdometryConstPtr &odom)
    {
        person1.x = odom->pose.pose.position.x;
        person1.y = odom->pose.pose.position.y;
    }

    void person2PosCallback(const nav_msgs::OdometryConstPtr &odom)
    {
        person2.x = odom->pose.pose.position.x;
        person2.y = odom->pose.pose.position.y;
    }*/


    //Compute the z coordinate in the world frame knowing both x and y

    double getZ(Point2d center, Point2d worldXY, Mat mapToCameraTransform)
    {
      Mat K = cameramodel->getK();
      Mat RT = mapToCameraTransform(Range(0,3), Range(0, 4));
      Mat P = K*RT;


      double z = worldXY.x*(center.x/center.y*P.at<double>(1,0)-P.at<double>(0,0))+worldXY.y*(center.x/center.y*P.at<double>(1,1)-P.at<double>(0,1))+center.x/center.y*P.at<double>(1,3)-P.at<double>(0,3);

      z = z/(P.at<double>(0,2)-center.x/center.y*P.at<double>(1,2));

      return z;
    }

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
            targetId = -1;
          }

      ROS_INFO_STREAM( feedback->marker_name << " is now at "
          << feedback->pose.position.x << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z );




    }

    void trackingCallback(const pedestrian_detector::DetectionList::ConstPtr &detection)
    {

      //Get rects from message

      vector<cv::Rect_<int> > rects;
      imageStamp = detection->header.stamp;

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

            //The following transforms are used if we want to calculate the position using a homography.

          stringstream camera;
          camera << cameraStr << "_vision_link";

          listener.waitForTransform("/map", camera.str(), ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform("/map", camera.str(),ros::Time(0), transform);
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

      cv::Mat mapToCameraTransform;

      invert(transform_opencv, mapToCameraTransform);

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

      coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrame(feetImagePoints, mapToCameraTransform);


      personList.associateData(coordsInBaseFrame, rects);


      //Delete the trackers that need to be deleted...
      for(vector<PersonModel>::iterator it = personList.personList.begin(); it != personList.personList.end();)
      {
          if(it->toBeDeleted)
          {
              if(it->id == targetId)
              {
                  personNotChosenFlag = true;
                  it = personList.personList.erase(it);
                  targetId = -1;

                  //Send eyes to home position
                  move_robot_msgs::GazeGoal fixationGoal;
                  fixationGoal.type = fixationGoal.HOME;
                  ac.sendGoal(fixationGoal);
                  ROS_INFO("Lost target. Sending eyes to home position");
              }
              else
              {
                it = personList.personList.erase(it);
              }
          }
          else
          {
              it++;
          }

      }



      vector<PersonModel> list = personList.getValidTrackerPosition();

      //Send the rects correctly ordered and identified back to the detector so that we can view it on the image

       /*
        *  TODO!
        *
        */


      //SIMULATION ONLY
      /*Write simulated persons positions to file. Frame | Id | x | y |*/
      /*Person1 ID: -1, Person2 ID: -2*/

//      results << frame << " " << "-1 " << person1.x << " " << person1.y << endl;
//      results << frame << " " << "-2 " << person2.x << " " << person2.y << endl;

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

//            results << frame << " " << (*it).id << " " << position.x << " " << position.y << endl;

            marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));
        }

        marker_server->applyChanges();
      }
      else
      {

        marker_server->clear();
        for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
        {

            Point3d position = (*it).medianFilter();

            if((*it).id == targetId)
            {

                //Start looking at that person. Even if we have to turn the base to avoid obstacles, we will still try to see
              // our target

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
              geometry_msgs::PointStamped final_position;

              final_position.header.stamp = imageStamp;
              final_position.point.x = position.x;
              final_position.point.y = position.y;
              final_position.point.z = 0;

              position_publisher.publish(final_position);


              move_robot_msgs::GazeGoal fixationGoal;

              //We wish to gaze at the center of the bounding box
              Point2d bbCenter = getCenter(it->rect);

              double z = getZ(bbCenter, Point2d(position.x, position.y), mapToCameraTransform);

              if(cv::norm(Point3d(position.x, position.y, z)-lastFixationPoint) > 0.5)
              {
                fixationGoal.fixation_point.header.frame_id="map";
                fixationGoal.fixation_point.point.x = position.x;
                fixationGoal.fixation_point.point.y =  position.y;
                fixationGoal.fixation_point.point.z = z;
                fixationGoal.fixation_point_error_tolerance = 0.5;

                fixationGoal.fixation_point.header.stamp=imageStamp;

                ac.sendGoal(fixationGoal);
                ROS_INFO("Gaze Action server started, sending goal.");

                lastFixationPoint = Point3d(position.x, position.y, z);

              }



              //wait for the action to return
/*              bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

              if (finished_before_timeout)
              {
                  actionlib::SimpleClientGoalState state = ac.getState();
                  ROS_INFO("Gaze action finished: %s",state.toString().c_str());
              }
              else
                  ROS_INFO("Gaze action did not finish before the time out.");
*/

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

      frame++;
    }
    Tracker(string cameraConfig) : ac("gaze", true), nPriv("~")
    {
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();

        nPriv.param<std::string>("camera", cameraStr, "l_camera");
        nPriv.param<std::string>("map_topic", mapTopic, "/map");

      personNotChosenFlag = true;

      //Initialize at infinity
      lastFixationPoint = Point3d(1000, 1000, 1000);

      cameramodel = new cameraModel(cameraConfig, cameraStr);
      ROS_ERROR("Subscribing detections");
      image_sub = n.subscribe("detections", 1, &Tracker::trackingCallback, this);
      ROS_ERROR("Subscribed");

      //Stuff for results...
 //     person1Topic = n.subscribe("/person1/odom", 1, &Tracker::person1PosCallback, this);
 //     person2Topic = n.subscribe("/person2/odom", 1, &Tracker::person2PosCallback, this);


      //Prepare the marker
      marker_server = new interactive_markers::InteractiveMarkerServer("tracker");

      visualization_msgs::Marker person_marker;

      person_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      person_marker.mesh_resource = "package://pedestrian_detector/meshes/animated_walking_man.mesh";
      person_marker.scale.x = 1.2 / 7.0 * 1.8;  //0.3
      person_marker.scale.y = 1.2 / 7.0 * 1.8; //0.3
      person_marker.scale.z = 1.2 / 7.0 * 1.8;  //1
      person_marker.color.r = 0;
      person_marker.color.g = 1;
      person_marker.color.b = 0;
      person_marker.color.a = 1.0;
      person_marker.pose.position.z = 0;

      person_marker.pose.orientation.x = 1;
      person_marker.pose.orientation.y = 0;
      person_marker.pose.orientation.z = 0;
      person_marker.pose.orientation.w = 1;

      visualization_msgs::InteractiveMarkerControl click_me;
      click_me.always_visible = true;
      click_me.markers.push_back(person_marker);
      click_me.name = "click";
      click_me.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

      int_marker.header.frame_id = "/map";
      int_marker.header.stamp=ros::Time::now();

      int_marker.scale = 1.5;

      int_marker.controls.push_back(click_me);

      position_publisher = n.advertise<geometry_msgs::PointStamped>("person_position", 1);
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

  //Simulation results file
//  results.open("/home/avelino/results.txt");

  ss << ros::package::getPath("pedestrian_detector");
  ss << "/camera_model/config.yaml";

  Tracker tracker(ss.str());

  ros::spin();

//  results.close();

  return 0;
}
