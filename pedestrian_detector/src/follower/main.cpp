/*******************************************
*
*   Person follower
*
*******************************************/

//ROS includes
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/eigen.hpp"


//Our includes
#include "../include/follower/controller.hpp"

using namespace std;


class FollowerFSM
{

private:

    enum State
    {
        STOPPED,
        PLANNER,
        LOCALCONTROLLER
    };

    geometry_msgs::PointStamped personMapPosition;
    double distanceToPerson;

    State currentState;
    State nextState;
    ros::Rate *rate;
    ros::Subscriber sub;
    MoveBaseClient *ac;

    ros::NodeHandle n;

    //If we are not receiving any data for a specific ammount of time then we put the node on
    //hold mode and stop the navigation!
    //Try to do it with a Timer and a callback function!
    ros::Time lastReceivedPerson;
    bool hold;


    tf::TransformListener listener;

    void receiveInformation(const geometry_msgs::PointConstPtr &person)
    {

        geometry_msgs::PointStamped personInRobotBaseFrame;

        personMapPosition.header.stamp = ros::Time();
        personMapPosition.header.frame_id = "/odom";

        personMapPosition.point.x = person->x;
        personMapPosition.point.y = person->y;
        personMapPosition.point.z = person->z;

        /*Check distance to objective*/
        //Get person on robot frame
        try
        {
            listener.transformPoint("/base_footprint", personMapPosition, personInRobotBaseFrame);

            cv::Point3d point(personInRobotBaseFrame.point.x, personInRobotBaseFrame.point.y, personInRobotBaseFrame.point.z);
            distanceToPerson = cv::norm(point);
            ROS_INFO("Person distance: %f", distanceToPerson);
            hold = false;

        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

public:
    FollowerFSM()
    {
        hold = true;
        currentState = STOPPED;
        nextState = STOPPED;
        rate = new ros::Rate(10);
        ac = new MoveBaseClient("move_base", true);

        sub = n.subscribe("person_position", 1, &FollowerFSM::receiveInformation, this);

    }

    ~FollowerFSM()
    {
        delete ac;
        if(rate != NULL)
            delete rate;
    }


    void run()
    {


        while(ros::ok())
        {

            //If we are receiving data
            if(!hold)
            {
                switch(currentState)
                {
                case STOPPED:
                    ROS_INFO("STOPPED!");

                    //Next state
                    if(distanceToPerson > 3)
                    {
                        nextState = PLANNER;
                        delete rate;
                        rate = new ros::Rate(2);
                    }
                    else if(distanceToPerson < 3 && distanceToPerson > 2)
                    {
                        nextState = LOCALCONTROLLER;
                        delete rate;
                        rate = new ros::Rate(10);
                    }
                    else
                    {
                        nextState = STOPPED;
                        rate->sleep();
                    }

                    break;
                case PLANNER:
                    ROS_INFO("PLANNER");

                    if(distanceToPerson >= 2.5)
                    {
                        nextState = PLANNER;
                        //Get transforms
                        tf::StampedTransform transform;
                        try
                        {
                          listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
                          listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), transform);
                        }
                        catch(tf::TransformException ex)
                        {
                          ROS_ERROR("%s",ex.what());
                          ros::Duration(1.0).sleep();
                        }

                        Eigen::Affine3d eigen_transform;
                        tf::transformTFToEigen(transform, eigen_transform);


                        // convert matrix from Eigen to openCV
                        cv::Mat odomToRobotBase;
                        cv::eigen2cv(eigen_transform.matrix(), odomToRobotBase);

                        cv::Point3d personPoint;
                        personPoint.x = personMapPosition.point.x;
                        personPoint.y = personMapPosition.point.y;
                        personPoint.z = personMapPosition.point.z;
                        segwayController::moveBase(personPoint, odomToRobotBase, ac);
                        rate->sleep();
                    }
                    else if( distanceToPerson < 2.5 && distanceToPerson >= 2)
                    {
                        nextState = LOCALCONTROLLER;
                        ac->cancelAllGoals();
                        delete rate;
                        rate = new ros::Rate(10);
                    }

                    else if(distanceToPerson < 2)
                    {
                        nextState = STOPPED;
                        ac->cancelAllGoals();
                        delete rate;
                        rate = new ros::Rate(10);
                    }

                    break;
                case LOCALCONTROLLER:
                    ROS_INFO("LOCALCONTROLLER");
                    //TODO!
                    /*
               *  Local controller...
               *
               */
                    if(distanceToPerson >= 2 && distanceToPerson < 3)
                    {
                        nextState = LOCALCONTROLLER;

                    }
                    else if(distanceToPerson < 2)
                    {
                        nextState = STOPPED;
                    }
                    else if(distanceToPerson >= 3)
                    {


                        nextState = PLANNER;
                        delete rate;
                        rate = new ros::Rate(2);

                    }

                    break;
                }
            }
            ros::spinOnce();
            currentState = nextState;
        }
    }

};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "Follower");

    //*******************************************************************************
    //segwayController::moveBase(coordsInBaseFrame[0], odomToBaseLinkTransform, *ac);
    //*******************************************************************************

    FollowerFSM fsm;
    fsm.run();



    return 0;
}
