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
    ros::Subscriber image_sub;
    bool personNotChosenFlag;
    bool automatic;
    Point3d targetCoords;
    PersonList *personList;
    Point3d lastFixationPoint;
    DetectionFilter *detectionfilter;



    actionlib::SimpleActionClient<move_robot_msgs::GazeAction> ac;
    std_msgs::Header last_image_header;


    int targetId;

    //Marker stuff
    interactive_markers::InteractiveMarkerServer *marker_server;
    visualization_msgs::InteractiveMarker int_marker;

    visualization_msgs::InteractiveMarker diceMarker;


    //Message
    ros::Publisher position_publisher;

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
    std::string world_frame;
    ros::NodeHandle nPriv;
    double gaze_threshold;
    int median_window;
    double fixation_tolerance;
    int numberOfFramesBeforeDestruction;
    int numberOfFramesBeforeDestructionLocked;
    double associatingDistance;
    double minimum_person_height;
    double maximum_person_height;



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
            move_robot_msgs::GazeGoal fixationGoal;
            fixationGoal.type = move_robot_msgs::GazeGoal::HOME;
            fixationGoal.fixation_point.header.stamp=ros::Time::now();
            fixationGoal.fixation_point.header.frame_id=filtering_frame_id;
            ac.sendGoal(fixationGoal);
            ROS_INFO("No target selected. Sending eyes to home position");

        }

        ROS_INFO_STREAM( feedback->marker_name << " is now at "
                         << feedback->pose.position.x << ", " << feedback->pose.position.y
                         << ", " << feedback->pose.position.z );




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

        last_image_header = detection->header;
        //Get transforms
        tf::StampedTransform transform;

        Mat lastImage = cv_ptr->image;

        ros::Time currentTime = ros::Time(0);

        try
        {

            listener->waitForTransform(cameraFrameId, last_image_header.stamp, filtering_frame_id, currentTime, fixed_frame_id, ros::Duration(10) );
            listener->lookupTransform(cameraFrameId, last_image_header.stamp, filtering_frame_id, currentTime, fixed_frame_id, transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_WARN("%s",ex.what());
            //ros::Duration(1.0).sleep();
            return;
        }

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

        coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrame(feetImagePoints, mapToCameraTransform, rects);


        //Before we associate the data we need to filter invalid detections
        detectionfilter->filterDetectionsByPersonSize(coordsInBaseFrame, rects, mapToCameraTransform);


        personList->associateData(coordsInBaseFrame, rects, colorFeaturesList);


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
                    move_robot_msgs::GazeGoal fixationGoal;
                    fixationGoal.fixation_point.header.stamp=ros::Time::now();
                    fixationGoal.fixation_point.header.frame_id=filtering_frame_id;
                    fixationGoal.type = move_robot_msgs::GazeGoal::HOME;
                    ac.sendGoal(fixationGoal);
                    ROS_INFO("Lost target. Sending eyes to home position");
                }
                else
                {
                    it = personList->personList.erase(it);
                }
            }
            else
            {
                it++;
            }

        }



        vector<PersonModel> list = personList->getValidTrackerPosition();

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
        ros::Time now=ros::Time::now();

        //If in automatic mode

        if(automatic)
            if(personNotChosenFlag)
        {
            //If there are no persons, return
            if(list.size() < 1)
                return;

            //If there is no person chosen, choose the closest one.
            int_marker.header.stamp=currentTime;
            int_marker.header.frame_id=world_frame;

            double best = 100000000000000;
            int personID;

            for(vector<PersonModel>::iterator it = list.begin(); it != list.end(); it++)
            {

                Point3d position = (*it).medianFilter();

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

                Point3d position = (*it).medianFilter();

                int_marker.controls.at(0).markers.at(0).color.r = 0;
                int_marker.controls.at(0).markers.at(0).color.g = 1;
                int_marker.controls.at(0).markers.at(0).color.b = 0;


                geometry_msgs::PointStamped personInMap;
                try
                {
                    geometry_msgs::PointStamped personInBase;
                    personInBase.header.frame_id = filtering_frame_id;
                    personInBase.header.stamp = last_image_header.stamp;
                    personInBase.point.x = position.x;
                    personInBase.point.y = position.y;
                    personInBase.point.z = 0;


                    listener->waitForTransform(world_frame, currentTime, filtering_frame_id, currentTime, fixed_frame_id, ros::Duration(10) );
                    listener->transformPoint(markers_frame_id, currentTime, personInBase, fixed_frame_id, personInMap);
                }
                catch(tf::TransformException ex)
                {
                    ROS_WARN("%s",ex.what());
                    //ros::Duration(1.0).sleep();
                    return;
                }

                int_marker.pose.position.x = personInMap.point.x;
                int_marker.pose.position.y = personInMap.point.y;

                //            results << frame << " " << (*it).id << " " << position.x << " " << position.y << endl;

                marker_server->insert(int_marker, boost::bind(&Tracker::processFeedback, this, _1));
            }
        }
        else
        {
            int_marker.header.stamp=currentTime;
            int_marker.header.frame_id=world_frame;

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

                    geometry_msgs::PointStamped personInMap;
                    try
                    {
                        geometry_msgs::PointStamped personInBase;
                        personInBase.header.frame_id = filtering_frame_id;
                        personInBase.header.stamp = currentTime;
                        personInBase.point.x = position.x;
                        personInBase.point.y = position.y;
                        personInBase.point.z = position.z;

                        listener->waitForTransform(world_frame, currentTime, filtering_frame_id, currentTime, fixed_frame_id, ros::Duration(10) );
                        listener->transformPoint(markers_frame_id, currentTime, personInBase, fixed_frame_id, personInMap);
                    }
                    catch(tf::TransformException ex)
                    {
                        ROS_WARN("%s",ex.what());
                        //ros::Duration(1.0).sleep();
                        return;
                    }

                    int_marker.pose.position.x = personInMap.point.x;
                    int_marker.pose.position.y = personInMap.point.y;

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

                    position_publisher.publish(final_position);

                    double z = position.z;

                    // CONTROL GAZE
                    if(cv::norm(Point3d(position.x, position.y, z)-lastFixationPoint) > gaze_threshold)
                    {
                        move_robot_msgs::GazeGoal fixationGoal;
                        fixationGoal.fixation_point.header.stamp=currentTime;
                        fixationGoal.fixation_point.header.frame_id=filtering_frame_id;
                        fixationGoal.fixation_point.point.x = position.x;
                        fixationGoal.fixation_point.point.y = position.y;
                        fixationGoal.fixation_point.point.z = z;
                        fixationGoal.fixation_point_error_tolerance = fixation_tolerance;



                        ac.sendGoal(fixationGoal);
                        ROS_INFO("Gaze Action server started, sending goal.");

                        //bool finished_before_timeout = ac.waitForResult(ros::Duration(10));


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
                    geometry_msgs::PointStamped personInMap;

                    try
                    {
                        geometry_msgs::PointStamped personInBase;
                        personInBase.header.frame_id = filtering_frame_id;
                        personInBase.header.stamp = currentTime;
                        personInBase.point.x = position.x;
                        personInBase.point.y = position.y;
                        personInBase.point.z = 0;


                        listener->waitForTransform(world_frame, currentTime, filtering_frame_id, currentTime, fixed_frame_id, ros::Duration(10));
                        listener->transformPoint(markers_frame_id, currentTime, personInBase, fixed_frame_id, personInMap);
                    }
                    catch(tf::TransformException ex)
                    {
                        ROS_WARN("%s",ex.what());
                        //ros::Duration(1.0).sleep();
                        return;
                    }

                    int_marker.pose.position.x = personInMap.point.x;
                    int_marker.pose.position.y = personInMap.point.y;

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
            rectangle(lastImage, it->rect, Scalar(0, 255, 0), 2);
            ostringstream convert;
            convert << it->id;
            putText(lastImage, convert.str(), it->rect.tl(), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar_<int>(255,0,0), 2);
        }

        sensor_msgs::ImagePtr msgImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lastImage).toImageMsg();
        image_pub.publish(msgImage);
    }
    Tracker(string cameraConfig) : ac("gaze", true), nPriv("~"),
        listener(new tf::TransformListener(ros::Duration(2.0)))
    {
        ROS_ERROR("Waiting for action server to start.");
        //ac.waitForServer();

        //For debug purposes
        it = new image_transport::ImageTransport(n);
        image_pub = it->advertise("image_out_tracker", 1);
        //*****************************************

        nPriv.param<std::string>("camera", cameraFrameId, "l_camera_vision_link");
        nPriv.param<std::string>("camera_info_topic", cameraInfoTopic, "/vizzy/l_camera/camera_info");
        nPriv.param<std::string>("filtering_frame_id", filtering_frame_id, "/odom");
        nPriv.param<std::string>("fixed_frame_id", fixed_frame_id, "/base_footprint");
        nPriv.param<std::string>("markers_frame_id", markers_frame_id, "/map");
        nPriv.param<std::string>("world_frame", world_frame, "/map");
        nPriv.param("gaze_threshold", gaze_threshold, 0.2);
        nPriv.param("median_window", median_window, 5);
        nPriv.param("fixation_tolerance", fixation_tolerance, 0.1);
        nPriv.param("number_of_frames_before_destruction", numberOfFramesBeforeDestruction, 25);
        nPriv.param("number_of_frames_before_destruction_locked", numberOfFramesBeforeDestructionLocked, 35);
        nPriv.param("associating_distance", associatingDistance, 0.5);

        /*The tallest man living is Sultan Ksen (Turkey, b.10 December 1982) who measured 251 cm (8 ft 3 in) in Ankara,
         *Turkey, on 08 February 2011.*/
        nPriv.param("maximum_person_height", maximum_person_height, 2.51);

        /*Chandra was declared the shortest human adult ever documented and verified, measuring 21.51 in (54.64 cm).
         *Height confirmed by Guinness World Records.
         */
        nPriv.param("minimum_person_height", minimum_person_height, 0.55);

        personList = new PersonList(median_window,numberOfFramesBeforeDestruction, numberOfFramesBeforeDestructionLocked, associatingDistance);

        personNotChosenFlag = true;
        automatic = false;

        //Initialize at infinity
        lastFixationPoint = Point3d(1000, 1000, 1000);

        cameramodel = new cameraModel(cameraConfig, cameraInfoTopic);
        detectionfilter = new DetectionFilter(maximum_person_height, minimum_person_height, cameramodel);

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
    //results.open("/home/avelino/debug_avelino.txt");

    ss << ros::package::getPath("pedestrian_detector");
    ss << "/camera_model/config.yaml";

    Tracker tracker(ss.str());

    ros::spin();

    //results.close();

    return 0;
}
