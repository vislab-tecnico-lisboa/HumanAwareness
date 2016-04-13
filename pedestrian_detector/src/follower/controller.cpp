#include "../include/follower/controller.hpp"
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/LinearMath/Matrix3x3.h"
#include <iostream>
#include <cmath>
#include <geometry_msgs/Twist.h>



using namespace cv;
using namespace std;


void segwayController::proportionalController(cv::Point3d person, ros::Publisher cmdPub, double kv, double kalfa)
{
    //Person is in the base_link_frame so everything is simple
    geometry_msgs::Twist msg;


    double ro = norm(person);
    double alpha = atan2(person.y, person.x);

    double v = kv*ro;
    double omega = kalfa*alpha;

    if(v>1)
        v = 1.0;
    else if(v < -1)
        v = 0;

    if(omega > 1)
        omega = 1.0;
    else if(omega < -1)
        omega = -1.0;

    msg.linear.x = v;
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = omega;

    cmdPub.publish(msg);

}
