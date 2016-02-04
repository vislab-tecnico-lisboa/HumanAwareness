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
    ros::Timer timer;
    MoveBaseClient *ac;
    bool hold;

    ros::NodeHandle n;
    ros::NodeHandle nPriv;

    //If we are not receiving any data for a specific ammount of time then we put the node on
    //hold mode and stop the navigation!
    //Try to do it with a Timer and a callback function!
    int notReceivedNumber;

    //Params
    std::string world_frame;
    std::string robot_frame;
    std::string fixed_frame_id;
    double minimum_distance;
    double minimum_planner_distance;
    double distance_to_target;
    double planner_activation_distance;

    tf::TransformListener listener;

    void checkInfo(const ros::TimerEvent& event)
    {
        notReceivedNumber++;
        if(notReceivedNumber > 1)
        {
            hold = true;
            ROS_DEBUG("Hold is true");
        }
    }

    void receiveInformation(const geometry_msgs::PointStampedConstPtr &person)
    {

        notReceivedNumber = 0;
        hold = false;
        geometry_msgs::PointStamped personInRobotBaseFrame;
	
	personInRobotBaseFrame = *person;

        personMapPosition.header.stamp = person->header.stamp;
        personMapPosition.header.frame_id = world_frame;

              geometry_msgs::PointStamped personInMap;
             try
              {
              geometry_msgs::PointStamped personInBase;

              listener.waitForTransform(world_frame, person->header.stamp, person->header.frame_id, person->header.stamp, fixed_frame_id, ros::Duration(10.0) );
              listener.transformPoint(world_frame, person->header.stamp, personInRobotBaseFrame, fixed_frame_id, personMapPosition);
              }
              catch(tf::TransformException ex)
              {
            ROS_WARN("%s",ex.what());
            //ros::Duration(1.0).sleep();
            return;
              }

        /*Check distance to objective*/
        //Get person on robot frame
        try
        {
            listener.transformPoint(robot_frame, personMapPosition, personInRobotBaseFrame);

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
    FollowerFSM() : hold(true),
        currentState(STOPPED),
        nextState(STOPPED),
        nPriv("~")


    {
        //hold = true;
        //currentState = STOPPED;
        //nextState = STOPPED;
        rate = new ros::Rate(10);
        ac = new MoveBaseClient("move_base", true);

        nPriv.param<std::string>("world_frame", world_frame, "world");
        nPriv.param<std::string>("robot_frame", robot_frame, "robot");
        nPriv.param<std::string>("fixed_frame", fixed_frame_id, "odom");
        nPriv.param<double>("minimum_distance", minimum_distance, 2);
        nPriv.param<double>("planner_activation_distance", planner_activation_distance, 1.7);
        nPriv.param("minimum_planner_distance", minimum_planner_distance, 1.7);
        nPriv.param("distance_to_target", distance_to_target, 1.5);

        notReceivedNumber = 0;
        sub = n.subscribe("person_position", 1, &FollowerFSM::receiveInformation, this);

        timer = n.createTimer(ros::Duration(0.1), &FollowerFSM::checkInfo, this); //If we didn't receive any position on 2 seconds STOP the robot!
        timer.start();
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
            switch(currentState)
            {
            case STOPPED:
                ROS_ERROR("STOPPED!");

                //Next state
                if(distanceToPerson > planner_activation_distance && hold == false)
                {
                    nextState = PLANNER;
                    delete rate;
                    rate = new ros::Rate(10);
                }
                else if(distanceToPerson < planner_activation_distance && distanceToPerson > minimum_distance && hold == false)
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
                ROS_ERROR("PLANNER");

                if(distanceToPerson >= minimum_planner_distance && hold == false)
                {
                    nextState = PLANNER;
                    //Get transforms
                    tf::StampedTransform transform;
                    try
                    {
                        listener.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(10.0) );
                        listener.lookupTransform(robot_frame, world_frame,ros::Time(0), transform);
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
                    segwayController::moveBase(personPoint, odomToRobotBase, ac, distance_to_target);
                    rate->sleep();
                }
                else if( distanceToPerson < minimum_planner_distance && distanceToPerson >= minimum_distance && hold == false)
                {
                    nextState = LOCALCONTROLLER;
                    ac->cancelAllGoals();
                    delete rate;
                    rate = new ros::Rate(10);
                }

                else if(distanceToPerson < minimum_distance || hold == true)
                {
                    nextState = STOPPED;
                    ac->cancelAllGoals();
                    delete rate;
                    rate = new ros::Rate(10);
                }

                break;
            case LOCALCONTROLLER:
                ROS_ERROR("LOCALCONTROLLER");
                //TODO!
                /*
               *  Local controller...
               *
               */
                if(distanceToPerson >= minimum_distance && distanceToPerson < planner_activation_distance && hold == false)
                {
                    nextState = LOCALCONTROLLER;

                }
                else if(distanceToPerson < minimum_distance || hold == true)
                {
                    nextState = STOPPED;
                }
                else if(distanceToPerson >= planner_activation_distance)
                {


                    nextState = PLANNER;
                    delete rate;
                    rate = new ros::Rate(10);

                }

                break;
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
