#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

class segwayController
{

  public:

    //Calculates target coordinates and orientation and moves the base to them
    //using the /move_base topic
    static void moveBase(cv::Point3d person, cv::Mat odomToBaseLink, ros::Publisher &control_pub);

};

#endif // CONTROLLER_HPP
