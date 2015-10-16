#include "../include/follower/controller.hpp"
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/LinearMath/Matrix3x3.h"
#include <iostream>
#include <cmath>



using namespace cv;
using namespace std;

void segwayController::moveBase(cv::Point3d person, cv::Mat odomToBaseLink, MoveBaseClient *ac)
{


    Mat personMat(person);

    personMat.convertTo(personMat, CV_32FC1);

    Mat ones = Mat::ones(1, 1, CV_32FC1);

    personMat.push_back(ones);

    Mat baseLinkToOdom;
    invert(odomToBaseLink, baseLinkToOdom);

    odomToBaseLink.convertTo(odomToBaseLink, CV_32FC1);
    baseLinkToOdom.convertTo(baseLinkToOdom, CV_32FC1);

    //We set the robot desired orientation

    //Transform the point to the base_link Frame (this will not be needed if I get the points on this frame directly,
    //wich I do... but for testing purposes I will keep it this way

    personMat = odomToBaseLink*personMat;


    //The desired orientation will be a rotation of PHI around the Z axis. PHI is the angle between X_robot (axis) and (-P)
    //Desired orientation on base_link frame
    float phi = atan2(personMat.at<float>(1, 0), personMat.at<float>(0, 0));

    float bl[] = {cos(phi), -sin(phi), 0, sin(phi), cos(phi), 0, 0, 0, 1, 0, 0, 0, 0, 1};
    Mat orientationOnBaseLink(4, 3, CV_32FC1, bl);

    //Desired orientation on world frame
    Mat orientationOnWorldFrame = baseLinkToOdom*orientationOnBaseLink;

    //And finally we get it in terms of quaternions...
    tf::Matrix3x3 tmp;

    //There is no easy way to do this :'(
    tmp[0][0] = orientationOnWorldFrame.at<float>(0, 0);
    tmp[0][1] = orientationOnWorldFrame.at<float>(0, 1);
    tmp[0][2] = orientationOnWorldFrame.at<float>(0, 2);
    tmp[1][0] = orientationOnWorldFrame.at<float>(1, 0);
    tmp[1][1] = orientationOnWorldFrame.at<float>(1, 1);
    tmp[1][2] = orientationOnWorldFrame.at<float>(1, 2);
    tmp[2][0] = orientationOnWorldFrame.at<float>(2, 0);
    tmp[2][1] = orientationOnWorldFrame.at<float>(2, 1);
    tmp[2][2] = orientationOnWorldFrame.at<float>(2, 2);

    tf::Quaternion q;
    tmp.getRotation(q);

    //Goal! Benfica!
    float x;
    float y;

    x = (norm(personMat)-2.5)*cos(phi);
    y = (norm(personMat)-2.5)*sin(phi);

    x = copysign(x, personMat.at<float>(0,0));
    y = copysign(y, personMat.at<float>(1,0));

    float goalPoint[] = {x, y, 0, 1};

    Mat goal = Mat(4, 1, CV_32FC1, goalPoint);

    cout << "Goal: " << goal << endl;


    //Now we get the goal on the world frame (/odom)

    Mat goalPointOnWorldFrame = baseLinkToOdom*goal;
    goalPointOnWorldFrame = goalPointOnWorldFrame/goalPointOnWorldFrame.at<float>(3, 0);




    move_base_msgs::MoveBaseGoal goal_msg;

    goal_msg.target_pose.header.frame_id = "map";
    goal_msg.target_pose.header.stamp = ros::Time::now();

    goal_msg.target_pose.pose.orientation.x = q.getX();
    goal_msg.target_pose.pose.orientation.y = q.getY();
    goal_msg.target_pose.pose.orientation.z = q.getZ();
    goal_msg.target_pose.pose.orientation.w = q.getW();

    goal_msg.target_pose.pose.position.x = goalPointOnWorldFrame.at<float>(0,0);
    goal_msg.target_pose.pose.position.y = goalPointOnWorldFrame.at<float>(1,0);
    goal_msg.target_pose.pose.position.z = 0;

    cout << "SENDING TO: " << "[" << goal_msg.target_pose.pose.position.x <<  ", " << goal_msg.target_pose.pose.position.y << "]" << endl;
    cout << "Orientation" << "[" << q.getX() << ", " << q.getY() <<" ,"  << q.getZ() << ", " << q.getW() << "]" << endl;

    ROS_DEBUG("Waiting for action server to start.");
    // wait for the action server to start


    if(ac->waitForServer(ros::Duration(10)))
    {
        //with no duration will wait for infinite time
        ROS_DEBUG("Started.");
        //wait for the action to return
        ac->sendGoal(goal_msg);
        ROS_ERROR("WAIT FOR RESULT...");

//        bool finished_before_timeout = ac->waitForResult(ros::Duration(10));
//        If I do this a new goal wont be sent until it achieves the previous one...



        actionlib::SimpleClientGoalState state = ac->getState();

        /*Timeout to avoid being stuck here forever*/
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(2.0); // Timeout of 2 seconds

        ROS_INFO_STREAM("Waiting for planning");
        while(ros::Time::now() - start_time < timeout) {
            if(state != actionlib::SimpleClientGoalState::PENDING)
                break;
        }

        ROS_ERROR("PLANNING COMPLETE.");

        if (true) //TMP! Replace acording to the state!
        {
            actionlib::SimpleClientGoalState state = ac->getState();
            if(state.SUCCEEDED)
            {
                ROS_INFO("Action finished: %s",state.toString().c_str());
//                sleep(1.0); //HACK TO AVOID WRONG SENSORY DATA
            }
            else
            {
                ROS_WARN("Point not valid.");
            }
        }
        else
        {
            ROS_ERROR("Action did not finish before the time out.");
        }
    }
    else
    {
        ROS_INFO("Could not connect to server.");
    }
}
