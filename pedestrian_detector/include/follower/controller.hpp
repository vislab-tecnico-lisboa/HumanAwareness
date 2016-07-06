#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include "ros/ros.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class segwayController
{

  public:

    //Calculates target coordinates and orientation and moves the base to them
    //using the /move_base topic
    static void moveBase(cv::Point3d person, cv::Mat baseLinkToMap, MoveBaseClient *ac, double distance_to_target);
    static void proportionalController(cv::Point3d person, ros::Publisher cmdPub, double kv, double komega);


};




#endif // CONTROLLER_HPP
