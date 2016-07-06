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
#include "../include/tracker/filtersAndUtilities.hpp"

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

//Eigen includes
#include <Eigen/Eigenvalues>


//Custom messages
#include <pedestrian_detector/DetectionList.h>
#include <pedestrian_detector/BoundingBox.h>

//Other includes
#include <sstream>
#include <stack>
#include <iostream>
#include <string>

//Stuff for results from simulation...
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>

//Gaze control
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_robot_msgs/GazeAction.h>
#include <nav_msgs/Odometry.h>
using namespace std;


//Stuff for results from simulation...
cv::Point2d person1;
cv::Point2d person2;
int frame = 1;
//ofstream results;


class Tracker{

private:

    cameraModel *cameramodel;
    boost::shared_ptr<tf::TransformListener> listener;
    tf::StampedTransform transform;
    ros::NodeHandle n;
    ros::NodeHandle nPriv;
    actionlib::SimpleActionClient<move_robot_msgs::GazeAction> ac;

    ros::Subscriber image_sub;
    //ros::Subscriber odom_sub;
    bool personNotChosenFlag;
    bool automatic;
    Point3d targetCoords;
    PersonList *personList;
    Point3d lastFixationPoint;
    DetectionFilter *detectionfilter;


    // Odometry stuff
    double alpha_1, alpha_2, alpha_3, alpha_4;
    double d_thresh_,a_thresh_;
    bool tracking_initialized_;
    std_msgs::Header last_image_header;
    ros::Time last_odom_time;

    int targetId;

    //Marker stuff
    interactive_markers::InteractiveMarkerServer *marker_server;
    visualization_msgs::InteractiveMarker int_marker;

    visualization_msgs::InteractiveMarker diceMarker;


    //Message
    ros::Publisher position_publisher;
    ros::Publisher location_uncertainty;

    //Image for debug purposes
    image_transport::ImageTransport *it;
    image_transport::Publisher image_pub;

    //Things to get results...
    //    ros::Subscriber person1Topic;
    //    ros::Subscriber person2Topic;

    std::string cameraFrameId;
    std::string fixed_frame_id;
    std::string cameraInfoTopic;
    std::string markers_frame_id;
    std::string filtering_frame_id;
    std::string marker_frame_id;

    std::string odom_frame_id;
    double gaze_threshold;
    int median_window;
    double fixation_tolerance;
    int numberOfFramesBeforeDestruction;
    int numberOfFramesBeforeDestructionLocked;
    double associatingDistance;
    double minimum_person_height;
    double maximum_person_height;

    // Odometry auxiliars
    nav_msgs::Odometry last_odom_msg;
    double last_odom_yaw;
    bool first_odom_msg;
    double covariance_marker_scale_;

    bool sendHome()
    {
        move_robot_msgs::GazeGoal fixationGoal;
        fixationGoal.fixation_point.header.stamp=ros::Time::now();
        fixationGoal.fixation_point.header.frame_id=filtering_frame_id;
        fixationGoal.type = move_robot_msgs::GazeGoal::HOME;
        ac.sendGoal(fixationGoal);

        return true;
    }

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

    void processAutomaticFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        if(!automatic)
        {
            automatic = true;
            diceMarker.description = "Deactivate Automatic Tracking";
            diceMarker.controls.at(0).markers.at(0).color.r = 1;
            diceMarker.controls.at(0).markers.at(0).color.g = 0;
            diceMarker.controls.at(0).markers.at(0).color.b = 0;

        }
        else
        {
            automatic = false;
            diceMarker.description = "Activate Automatic Tracking";
            diceMarker.controls.at(0).markers.at(0).color.r = 0;
            diceMarker.controls.at(0).markers.at(0).color.g = 1;
            diceMarker.controls.at(0).markers.at(0).color.b = 0;
            personNotChosenFlag = true;
            sendHome();
        }

    }

    //Process the clicks on markers!
    void processFeedback(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {
        //If automatic tracking is activated we cant click on a specific person
        if(automatic)
            return;

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
            sendHome();

            ROS_INFO("No target selected. Sending eyes to home position");
        }

        ROS_INFO_STREAM( feedback->marker_name << " is now at "
                         << feedback->pose.position.x << ", " << feedback->pose.position.y
                         << ", " << feedback->pose.position.z );
    }

    void odometry()
    {

        tf::StampedTransform baseDeltaTf;

        ros::Time current_time=ros::Time::now();
        // First detection is discarded to use diffential times
        if(!tracking_initialized_)
        {
            try
            {
                listener->waitForTransform(fixed_frame_id, last_odom_time, fixed_frame_id, current_time , odom_frame_id, ros::Duration(0.5) );
                listener->lookupTransform(fixed_frame_id, last_odom_time, fixed_frame_id, current_time, odom_frame_id, baseDeltaTf); // delta position
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR("RETURN BEFORE INIT");
                return;
            }
            tracking_initialized_=true;
            last_odom_time = current_time;
            return;
        }



        // Get odom delta motion in cartesian coordinates with TF
        try
        {
            listener->waitForTransform(fixed_frame_id, current_time, fixed_frame_id, last_odom_time , odom_frame_id, ros::Duration(0.1) );
            listener->lookupTransform(fixed_frame_id, current_time, fixed_frame_id, last_odom_time, odom_frame_id, baseDeltaTf); // delta position

        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ROS_ERROR_STREAM("DEU RETURN");
            return;
        }
        last_odom_time=current_time;

        // Get control input
        double dx=baseDeltaTf.getOrigin().getX();
        double dy=baseDeltaTf.getOrigin().getY();
        double d_theta=baseDeltaTf.getRotation().getAxis()[2]*baseDeltaTf.getRotation().getAngle();

        // See if we should update the filter
        if(!(fabs(dx) > d_thresh_ || fabs(dy) > d_thresh_ || fabs(d_theta) > a_thresh_))
        {
            return;
        }

        double delta_rot1=atan2(dy,dx);
        double delta_trans=sqrt(dx*dx+dy*dy);
        double delta_rot2=d_theta-delta_rot1;

        double var_rot1=alpha_1*delta_rot1*delta_rot1+alpha_2*delta_trans*delta_trans;
        double var_trans=alpha_3*delta_trans*delta_trans+alpha_4*(delta_rot1*delta_rot1+delta_rot2*delta_rot2);
        double var_rot2=alpha_1*delta_rot2*delta_rot2+alpha_2*delta_trans*delta_trans;

        cv::Mat control_mean(3, 1, CV_64F);
        control_mean = (Mat_<double>(3, 1) << delta_rot1, delta_trans, delta_rot2);

        cv::Mat control_noise = Mat::zeros(3, 3, CV_64F);
        control_noise.at<double>(0,0)=var_rot1;
        control_noise.at<double>(1,1)=var_trans;
        control_noise.at<double>(2,2)=var_rot2;

        Mat odom_rot = (Mat_<double>(2, 2) << cos(d_theta), -sin(d_theta), sin(d_theta), cos(d_theta));
        Mat odom_trans = (Mat_<double>(2, 1) << dx*cos(d_theta), dy*sin(d_theta));

        // Linearize control noise
        cv::Mat J(2, 3, CV_64F);

        J.at<double>(0,0)=-sin(delta_rot1)*delta_trans;
        J.at<double>(0,1)=cos(delta_rot1);
        J.at<double>(0,2)=0;

        J.at<double>(1,0)=cos(delta_rot1)*delta_trans;
        J.at<double>(1,1)=sin(delta_rot1);
        J.at<double>(1,2)=0;

        // Odometry xy covariance
        cv::Mat R(2, 3, CV_64F);
        R=J*control_noise*J.t();

        for(std::vector<PersonModel>::iterator it = personList->personList.begin(); it!=personList->personList.end(); it++)
        {

            //Update the fused state
            for(std::vector<KalmanFilter>::iterator it_mmae=it->mmaeEstimator->filterBank.begin(); it_mmae!=it->mmaeEstimator->filterBank.end(); it_mmae++)
            {
                it_mmae->statePost(cv::Range(0,2),cv::Range(0,1)) = odom_trans + odom_rot*it_mmae->statePost(cv::Range(0,2),cv::Range(0,1));


                // Get number of blocks
                int row_blocks=it_mmae->errorCovPost.rows%2;
                int col_blocks=it_mmae->errorCovPost.cols%2;

                // For each block (position, velocity, acceleration
                for(int i=0;i<row_blocks;++i)
                {
                    for(int j=0;i<col_blocks;++i)
                    {
                        // Rotate covariance matrix
                        it_mmae->errorCovPost(cv::Range(i*2,(i+1)*2),cv::Range(j*2,(j+1)*2)) = (odom_rot*it_mmae->errorCovPost(cv::Range(i*2,(i+1)*2),cv::Range(j*2,(j+1)*2))*odom_rot.t());
                    }
                }

                // Add noise (xy position only, no velocities for now)
                it_mmae->errorCovPost(cv::Range(0,2),cv::Range(0,2))+=R;
            }
        }
    }

    void drawCovariances()
    {
        //For each person
        for(std::vector<PersonModel>::iterator it = personList->personList.begin(); it!=personList->personList.end(); it++)
        {
            int lol = 0;
            //For each motion model of a person
            for(std::vector<KalmanFilter>::iterator it_mmae=it->mmaeEstimator->filterBank.begin(); it_mmae!=it->mmaeEstimator->filterBank.end(); it_mmae++, lol++)
            {
                // Add noise (xy position only, no velocities for now)

                Mat errorCovPost = it_mmae->errorCovPost(cv::Range(0,2),cv::Range(0,2));

                errorCovPost.convertTo(errorCovPost, CV_32FC1);

                Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> covMatrix;

                cv2eigen(errorCovPost, covMatrix);

                visualization_msgs::Marker tempMarker;
                tempMarker.pose.position.x = it_mmae->statePost.at<double>(0,0);
                tempMarker.pose.position.y = it_mmae->statePost.at<double>(1,0);

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

                const Eigen::Vector2f& eigValues (eig.eigenvalues());
                const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

                float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));


                tempMarker.type = visualization_msgs::Marker::SPHERE;

                double lengthMajor = sqrt(eigValues[0]);
                double lengthMinor = sqrt(eigValues[1]);

                tempMarker.scale.x = covariance_marker_scale_*lengthMajor;
                tempMarker.scale.y = covariance_marker_scale_*lengthMinor;
                tempMarker.scale.z = 0.001;

                tempMarker.color.a = 1.0;
                tempMarker.color.r = 1.0;

                tempMarker.pose.orientation.w = cos(angle*0.5);
                tempMarker.pose.orientation.z = sin(angle*0.5);

                tempMarker.header.frame_id=fixed_frame_id;
                tempMarker.id = (*it).id;
                tempMarker.lifetime=ros::Duration(0.1);
                location_uncertainty.publish(tempMarker);
            }

        }
    }


    void trackingCallback(const pedestrian_detector::DetectionList::ConstPtr &detection)
    {


        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(detection->im, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //Get transforms
        tf::StampedTransform transform;

        Mat lastImage = cv_ptr->image;

        ros::Time currentTime = ros::Time(0);



        //ROS_ERROR_STREAM("Getting transform at 228");

        try
        {

            listener->waitForTransform(cameraFrameId, detection->header.stamp, filtering_frame_id, currentTime, fixed_frame_id, ros::Duration(0.1) );
            listener->lookupTransform(cameraFrameId, detection->header.stamp, filtering_frame_id, currentTime, fixed_frame_id, transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_WARN("%s",ex.what());
            //ros::Duration(1.0).sleep();
            return;
        }

        last_image_header = detection->header;




        //Get rects from message

        vector<cv::Rect_<int> > rects;
        vector<Mat> colorFeaturesList;


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


        for(pedestrian_detector::DetectionList::_featuresVector_type::const_iterator it = detection->featuresVector.begin(); it != detection->featuresVector.end(); it++)
        {
            vector<float> features = it->features;
            Mat bvtHistogram(features);

            colorFeaturesList.push_back(bvtHistogram.clone());
        }


        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(transform, eigen_transform);

        // convert matrix from Eigen to openCV
        cv::Mat mapToCameraTransform;
        cv::eigen2cv(eigen_transform.matrix(), mapToCameraTransform);

        //invert(transform_opencv, mapToCameraTransform);

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


        //ROS_ERROR_STREAM("Computing points on world frame");
        coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrame(feetImagePoints, mapToCameraTransform, rects);
        //ROS_ERROR_STREAM("Computed");

        //Before we associate the data we need to filter invalid detections
        detectionfilter->filterDetectionsByPersonSize(coordsInBaseFrame, rects, mapToCameraTransform);

        //ROS_ERROR_STREAM("Associating data");
        personList->associateData(coordsInBaseFrame, rects, colorFeaturesList);
        //ROS_ERROR_STREAM("Done");

        //Delete the trackers that need to be deleted...
        for(vector<PersonModel>::iterator it = personList->personList.begin(); it != personList->personList.end();)
        {
            if(it->toBeDeleted)
            {
                if(it->id == targetId)
                {
                    personNotChosenFlag = true;
                    it = personList->personList.erase(it);
                    targetId = -1;

                    //Send eyes to home position
                    sendHome();
                    ROS_INFO("Lost target. Sending eyes to home position");
                }
                else
                {
                    it = personList->personList.erase(it);
                }
            }
            else
            {
                //results << frame << " " << (*it).id << " " << it->positionHistory[0].x << " " << it->positionHistory[0].y << endl;
                it++;
            }

        }
        vector<PersonModel> list = personList->getValidTrackerPosition();
        /*        for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
        {

            geometry_msgs::PointStamped personInBase;
            geometry_msgs::PointStamped personInBaseFootprint;
            personInBase.header.frame_id = filtering_frame_id;
            personInBase.header.stamp = currentTime;
            personInBase.point.x = it->positionHistory[0].x;
            personInBase.point.y = it->positionHistory[0].y;
            personInBase.point.z = it->positionHistory[0].z;

            try{
                listener->waitForTransform("base_footprint", currentTime, filtering_frame_id, currentTime, fixed_frame_id, ros::Duration(10) );
                listener->transformPoint("base_footprint", currentTime, personInBase, fixed_frame_id, personInBaseFootprint);
            }catch(tf::TransformException ex)
            {
                ROS_WARN("%s",ex.what());
                //ros::Duration(1.0).sleep();
                return;
            }

            results << it->id << " " << personInBaseFootprint.point.x << " " << personInBaseFootprint.point.z*2 << endl;
        }*/


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
        marker_server->clear();
        marker_server->insert(diceMarker, boost::bind(&Tracker::processAutomaticFeedback, this, _1));
        //ros::Time now=ros::Time::now();

        //If in automatic mode
        if(automatic)
            if(personNotChosenFlag)
            {
                //If there are no persons, return
                if(list.size() < 1)
                    return;

                //If there is no person chosen, choose the closest one.
                int_marker.header.stamp=currentTime;
                int_marker.header.frame_id=filtering_frame_id;

                double best = 100000000000000;
                int personID=-1;

                for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
                {

                    Point3d position = (*it).getPositionEstimate();

                    //WARNING!
                    //Assuming the position is relative to base_footprint. Avoiding tf's for computational purposes.

                    double dist = cv::norm(position);
                    if(dist < best)
                    {
                        best = dist;
                        personID = it-> id;
                    }
                }

                targetId = personID;
                personNotChosenFlag = false;
            }



        if(personNotChosenFlag)
        {

            for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
            {
                stringstream description, name;
                name << "person " << (*it).id;
                description << "Detection " << (*it).id;
                int_marker.header.stamp=currentTime;
                int_marker.name = name.str();
                int_marker.description = description.str();

                Point3d position = (*it).getPositionEstimate();

                int_marker.controls.at(0).markers.at(0).color.r = 0;
                int_marker.controls.at(0).markers.at(0).color.g = 1;
                int_marker.controls.at(0).markers.at(0).color.b = 0;



                //ROS_ERROR_STREAM("Getting transform at 448");
                geometry_msgs::PointStamped personInBase;

                try
                {

                    personInBase.header.frame_id = filtering_frame_id;
                    personInBase.header.stamp = last_image_header.stamp;
                    personInBase.point.x = position.x;
                    personInBase.point.y = position.y;
                    personInBase.point.z = 0;


                }
                catch(tf::TransformException ex)
                {
                    ROS_WARN("%s",ex.what());
                    //ros::Duration(1.0).sleep();
                    return;
                }

                //results << frame << " " << (*it).id << " " << position.x << " " << position.y << endl;

                marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));
            }
        }
        else
        {
            int_marker.header.stamp=currentTime;
            int_marker.header.frame_id=filtering_frame_id;

            for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); ++it)
            {
                Point3d position = (*it).getPositionEstimate();

                if((*it).id == targetId)
                {
                    //Start looking at that person. Even if we have to turn the base to avoid obstacles, we will still try to see
                    // our target

                    it->lockedOnce = true;

                    int_marker.controls.at(0).markers.at(0).color.r = 1;
                    int_marker.controls.at(0).markers.at(0).color.g = 0;
                    int_marker.controls.at(0).markers.at(0).color.b = 0;



                    //ROS_ERROR_STREAM("Getting transform at 504");
                    geometry_msgs::PointStamped personInBase;
                    try
                    {

                        personInBase.header.frame_id = filtering_frame_id;
                        personInBase.header.stamp = currentTime;
                        personInBase.point.x = position.x;
                        personInBase.point.y = position.y;
                        personInBase.point.z = position.z;


                    }
                    catch(tf::TransformException ex)
                    {
                        ROS_WARN("%s",ex.what());
                        //ros::Duration(1.0).sleep();
                        return;
                    }

                    //ROS_ERROR_STREAM("Got 504");

                    int_marker.pose.position.x = personInBase.point.x;
                    int_marker.pose.position.y = personInBase.point.y;

                    stringstream description, name;
                    name << "person " << (*it).id;
                    description << "Objective: Detection " << (*it).id;

                    int_marker.name = name.str();
                    int_marker.description = description.str();

                    marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));

                    //Now we send out the position
                    geometry_msgs::PointStamped final_position;

                    final_position.header.stamp = currentTime;
                    final_position.header.frame_id = filtering_frame_id;

                    final_position.point.x = position.x;
                    final_position.point.y = position.y;
                    final_position.point.z = 0;

                    // CONTROL GAZE
                    if(personInBase.point.x > 0.5)
                    {
                        move_robot_msgs::GazeGoal fixationGoal;
                        fixationGoal.fixation_point.header.stamp=currentTime;
                        fixationGoal.fixation_point.header.frame_id=filtering_frame_id;
                        fixationGoal.fixation_point.point.x = position.x;
                        fixationGoal.fixation_point.point.y = position.y;
                        fixationGoal.fixation_point.point.z = it->personHeight/2;
                        fixationGoal.fixation_point_error_tolerance = fixation_tolerance;



                        ac.sendGoal(fixationGoal);
                        ROS_INFO("Gaze Action server started, sending goal.");

                        //bool finished_before_timeout =
                        ac.waitForResult(ros::Duration(2));


                        lastFixationPoint = Point3d(position.x, position.y, it->personHeight/2);

                    }


                    position_publisher.publish(final_position);

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

                    Point3d position = (*it).getPositionEstimate();

                    geometry_msgs::PointStamped personInBase;
                    //ROS_ERROR_STREAM("Getting transform at 600");
                    try
                    {

                        personInBase.header.frame_id = filtering_frame_id;
                        personInBase.header.stamp = currentTime;
                        personInBase.point.x = position.x;
                        personInBase.point.y = position.y;
                        personInBase.point.z = 0;

                    }
                    catch(tf::TransformException ex)
                    {
                        ROS_WARN("%s",ex.what());
                        //ros::Duration(1.0).sleep();
                        return;
                    }


                    int_marker.pose.position.x = personInBase.point.x;
                    int_marker.pose.position.y = personInBase.point.y;

                    stringstream description, name;

                    name << "person " << (*it).id;
                    description << "Detection " << (*it).id;

                    int_marker.name = name.str();
                    int_marker.description = description.str();

                    marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));

                }
            }
        }
        marker_server->applyChanges();
        frame++;

        //Write rectangles on image with ids and publish

        for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
        {

            //Get feet position
            Point3d head = it->getPositionEstimate();

            Point3d feet = head;
            feet.z = 0;

            //Project them into the image
            Point2d feetOnImage;
            Point2d headOnImage;

            /*Code*/
            Mat headMat = Mat::ones(4, 1, CV_32F);
            headMat.at<float>(0,0) = head.x;
            headMat.at<float>(1,0) = head.y;
            headMat.at<float>(2,0) = head.z;

            Mat feetMat = Mat::ones(4, 1, CV_32F);
            feetMat.at<float>(0,0) = feet.x;
            feetMat.at<float>(1,0) = feet.y;
            feetMat.at<float>(2,0) = feet.z;

            Mat intrinsics = cameramodel->getK();
            intrinsics.convertTo(intrinsics, CV_32F);

            Mat RtMat = mapToCameraTransform(Range(0, 3), Range(0, 4));
            RtMat.convertTo(RtMat, CV_32F);

            Mat feetOnImageMat = intrinsics*RtMat*feetMat;
            Mat headOnImageMat = intrinsics*RtMat*headMat;

            feetOnImage.x = feetOnImageMat.at<float>(0,0)/feetOnImageMat.at<float>(2,0);
            feetOnImage.y = feetOnImageMat.at<float>(1,0)/feetOnImageMat.at<float>(2,0);

            headOnImage.x = headOnImageMat.at<float>(0,0)/headOnImageMat.at<float>(2,0);
            headOnImage.y = headOnImageMat.at<float>(1,0)/headOnImageMat.at<float>(2,0);

            //Draw the rectangle from the point

            int h = feetOnImage.y-headOnImage.y;
            int width = h*52/128;

            int topLeftX = headOnImage.x - width/2;
            int topLeftY = headOnImage.y;

            cv::Rect_<int> trackedBB(topLeftX, topLeftY, width, h);

            if(h > 0)
            {

                //Bar plot of probabilities

                int step = trackedBB.width/3;
                int maxBarSize = trackedBB.height*4/5;
                const static Scalar cores[3] = {Scalar(24, 54, 231), Scalar(239, 232, 31), Scalar(255, 0, 0)};

                int ind = 0;
                Point2d barBase(trackedBB.tl().x, trackedBB.br().y); //Bottom left corner of the bb
                for(std::vector<double>::iterator pr = it->mmaeEstimator->probabilities.begin(); pr != it->mmaeEstimator->probabilities.end(); pr++)
                {
                    Point2d tl = barBase-Point2d(0, maxBarSize*(*pr));
                    rectangle(lastImage, tl, barBase + Point2d(step, 0), cores[ind], CV_FILLED);

                    if(ind < 1)
                        ind++;
                    else
                        ind = 0;

                    barBase = barBase + Point2d(step, 0);
                }

                ostringstream convert;

                if(it->id == targetId)
                {
                    rectangle(lastImage, trackedBB, Scalar(0, 0, 255), 2);
                    convert << "id: " << it->id << " | H = " << head.z;

                }else{
                    rectangle(lastImage, trackedBB, Scalar(0, 255, 0), 2);
                    convert << "id: " << it->id;
                }

                putText(lastImage, convert.str(), trackedBB.tl(), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar_<int>(255,0,0), 2);

            }
        }

        sensor_msgs::ImagePtr msgImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lastImage).toImageMsg();
        image_pub.publish(msgImage);


    }
    Tracker(string cameraConfig) : listener(new tf::TransformListener(ros::Duration(2.0))), ac("gaze", true), nPriv("~"),tracking_initialized_(false)
    {
        ROS_INFO("Waiting for action server to start.");
        //ac.waitForServer();

        //For debug purposes
        it = new image_transport::ImageTransport(n);
        image_pub = it->advertise("image_out_tracker", 1);
        //*****************************************

        nPriv.param<std::string>("camera", cameraFrameId, "l_camera_vision_link");
        nPriv.param<std::string>("camera_info_topic", cameraInfoTopic, "/vizzy/l_camera/camera_info");
        nPriv.param<std::string>("filtering_frame_id", filtering_frame_id, "/base_footprint");
        nPriv.param<std::string>("fixed_frame_id", fixed_frame_id, "/base_footprint");
        nPriv.param<std::string>("markers_frame_id", markers_frame_id, "/base_footprint");

        nPriv.param<std::string>("odom_frame_id", odom_frame_id, "/odom");

        nPriv.param("gaze_threshold", gaze_threshold, 0.2);
        nPriv.param("median_window", median_window, 5);
        nPriv.param("fixation_tolerance", fixation_tolerance, 0.1);
        nPriv.param("number_of_frames_before_destruction", numberOfFramesBeforeDestruction, 25);
        nPriv.param("number_of_frames_before_destruction_locked", numberOfFramesBeforeDestructionLocked, 35);
        nPriv.param("associating_distance", associatingDistance, 0.5);
        nPriv.param("alpha_1",alpha_1, 0.05);
        nPriv.param("alpha_2",alpha_2, 0.001);
        nPriv.param("alpha_3",alpha_3, 5.0);
        nPriv.param("alpha_4",alpha_4, 0.05);
        nPriv.param("update_min_d", d_thresh_, 0.2);
        nPriv.param("update_min_a", a_thresh_, M_PI/100.0);
        nPriv.param("covariance_marker_scale", covariance_marker_scale_, 2.0);

        /*The tallest man living is Sultan Ksen (Turkey, b.10 December 1982) who measured 251 cm (8 ft 3 in) in Ankara,
         *Turkey, on 08 February 2011.*/
        nPriv.param("maximum_person_height", maximum_person_height, 2.51);

        /*Chandra was declared the shortest human adult ever documented and verified, measuring 21.51 in (54.64 cm).
         *Height confirmed by Guinness World Records.
         */
        nPriv.param("minimum_person_height", minimum_person_height, 0.55);

        personList = new PersonList(median_window,numberOfFramesBeforeDestruction, numberOfFramesBeforeDestructionLocked, associatingDistance, MMAETRACKING);

        personNotChosenFlag = true;
        automatic = false;

        //Initialize at infinity
        lastFixationPoint = Point3d(1000, 1000, 1000);

        cameramodel = new cameraModel(cameraConfig, cameraInfoTopic);
        detectionfilter = new DetectionFilter(maximum_person_height, minimum_person_height, cameramodel);

        ROS_INFO("Subscribing detections");
        image_sub = n.subscribe("detections", 1, &Tracker::trackingCallback, this);
        ROS_INFO("Subscribed");

        //odom_sub=n.subscribe("odom", 1, &Tracker::odometryCallback, this);


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

        int_marker.header.frame_id = markers_frame_id;
        int_marker.scale = 1.5;

        int_marker.controls.push_back(click_me);



        visualization_msgs::Marker diceRoll;

        diceRoll.header.frame_id = "/base_footprint";
        diceRoll.header.stamp = ros::Time();
        diceRoll.id = 1;
        diceRoll.ns = "automatic";
        diceRoll.type = visualization_msgs::Marker::MESH_RESOURCE;
        diceRoll.mesh_resource = "package://pedestrian_detector/meshes/dice.stl";
        diceRoll.mesh_use_embedded_materials = false;
        diceRoll.color.r = 0;
        diceRoll.color.g = 1;
        diceRoll.color.b = 0;
        diceRoll.color.a = 1;

        diceRoll.scale.x = 1.2 / 7.0 * 1.8;  //0.3
        diceRoll.scale.y = 1.2 / 7.0 * 1.8; //0.3
        diceRoll.scale.z = 1.2 / 7.0 * 1.8;  //1
        diceRoll.pose.position.z = 2;
        diceRoll.pose.position.x = 0;
        diceRoll.pose.position.y = 0;

        diceRoll.pose.orientation.x = 1;
        diceRoll.pose.orientation.y = 0;
        diceRoll.pose.orientation.z = 0;
        diceRoll.pose.orientation.w = 1;

        visualization_msgs::InteractiveMarkerControl click_dice;

        click_dice.always_visible = true;
        click_dice.markers.push_back(diceRoll);
        click_dice.name = "clickDice";
        click_dice.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

        diceMarker.header.frame_id = "/base_footprint";
        diceMarker.scale = 1.5;

        diceMarker.controls.push_back(click_dice);
        diceMarker.pose.position.x = 0;
        diceMarker.pose.position.y = 0;
        diceMarker.pose.position.z = 0;

        diceMarker.description = "Activate Automatic Tracking";


        position_publisher = n.advertise<geometry_msgs::PointStamped>("person_position", 1);
        location_uncertainty = n.advertise<visualization_msgs::Marker>( "uncertainty_marker", 0 );
    }

    ~Tracker()
    {
        delete cameramodel;
        delete marker_server;
        delete personList;
        delete detectionfilter;
    }

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracker");

    stringstream ss;

    //Simulation results file
    //results.open("/home/avelino/altura_z.txt");

    ss << ros::package::getPath("pedestrian_detector");
    ss << "/camera_model/config.yaml";

    Tracker tracker(ss.str());

    ros::Rate r(200);

    while(ros::ok())
    {
        // Account for what the robot moved
        tracker.odometry();
        ros::spinOnce();
        r.sleep();
        tracker.drawCovariances();
    }

    //results.close();

    return 0;
}

