/*******************************************
*
*   Pedestrian Detector
*
*******************************************/

//General purpose includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <stack>
#include <string>
#include <sstream>
//#include <thread>
#include <fstream>

//ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Our detector
#include "../include/detector/pedestrianDetector.hpp"

//Our custom messages
#include <pedestrian_detector/DetectionList.h>
#include <pedestrian_detector/BoundingBox.h>
#include <pedestrian_detector/Features.h>

#include "../include/detector/colorFeatures.hpp"

//Detection options: pedestrian
//headandshoulders
//full


using namespace std;
using namespace cv;

int movAverageLength=10;
float currentTime = 0, oldestTime = 0, timesSum = 0;
std::deque<float> times (movAverageLength,0); //I keep the 10 most recent processing times, to compute a moving average
int frameCounter = 0;

std::vector<cv::Rect> partMasks;

//ofstream myfile;

class PedDetector
{

private:
    //ROS NodeHandle, and image_transport objects
    ros::NodeHandle nh;
    ros::NodeHandle nPriv;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    std::string detectorType;

    //Our detectors
    pedestrianDetector *person_detector;

    //Detections publisher
    ros::Publisher detectionPublisher;

    //Callback to process the images
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        tic();

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image(cv_ptr->image);



        person_detector->runDetector(image);
        Mat imageDisplay = image.clone();



        pedestrian_detector::DetectionList detectionList;

        detectionList.header = msg->header;
        detectionList.header.frame_id = "l_camera_vision_link";


        //Print rectangles on the image and add them to the detection list
        //Print detection numbers so that we can initialize the tracker using Rviz
        //And send a marker with each detection number to Rviz
        vector<cv::Rect_<int> >::iterator it;

        if(detectorType.compare("pedestrian") == 0 || detectorType.compare("full") == 0)
        {
            for(it = person_detector->boundingBoxes->begin(); it != person_detector->boundingBoxes->end(); it++){

                Mat person = image(*it);
                Mat resizedPerson;
                //Resize it for 52x128

                resize(person, resizedPerson, Size(52, 128));

                Mat bvtHistogram;

                extractBVT(resizedPerson, bvtHistogram, 10, partMasks);

                bvtHistogram.convertTo(bvtHistogram, CV_32FC1);


                //Most efficient way to convert a row Mat to std::vector, according to stackoverflow
                const float* p = bvtHistogram.ptr<float>(0);

                // Copy data to a vector.  Note that (p + mat.cols) points to the
                // end of the row.
                std::vector<float> vec(p, p + bvtHistogram.cols);

                pedestrian_detector::Features colorFeatures;

                colorFeatures.features = vec;

                rectangle(imageDisplay, *it, Scalar_<int>(0,255,0), 3);

                pedestrian_detector::BoundingBox bb;
                bb.x = (*it).x;
                bb.y = (*it).y;
                bb.width = (*it).width;
                bb.height = (*it).height;
                detectionList.bbVector.push_back(bb);
                detectionList.featuresVector.push_back(colorFeatures);
            }
        }


        //Compute processing time
        frameCounter++;
        currentTime=tocMatteo();
        times.push_front(currentTime); //Insert a new time in the list
        oldestTime = times.back(); //Read the oldest time from the list
        times.pop_back(); //Remove the oldest time from the list
        //Update the moving average sum
        timesSum+=currentTime;
        timesSum-=oldestTime;
        if(frameCounter>=10)
        {
            //          cout<<"Processing time = "<<timesSum/float(movAverageLength)<<". FPS="<<float(movAverageLength)/timesSum<<std::endl;
            stringstream ss;
            ss << float(movAverageLength)/timesSum;
            putText(imageDisplay, ss.str(), Point(0, 30), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar_<int>(255,0,0), 2);
        }

        //Number of detections, fps
        //        myfile << person_detector->headBoundingBoxes->size() << " " << float(movAverageLength)/timesSum << endl;


        if(detectorType.compare("headandshoulders") == 0 || detectorType.compare("full") == 0)
        {
            for(it = person_detector->headBoundingBoxes->begin(); it != person_detector->headBoundingBoxes->end(); it++)
            {
                rectangle(imageDisplay, *it, Scalar_<int>(0,0,255), 3);
            }
        }

        //Publish the resulting image for now. Later I might publish only the detections for another node to process
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageDisplay).toImageMsg();
        image_pub.publish(out_msg);





        //Publish the detections (I will also publish the features associated to each detection)
        detectionList.im = *msg;
        detectionPublisher.publish(detectionList);

    }

public:
    PedDetector(ros::NodeHandle & nh_,string conf_pedestrians, string conf_heads): nh(nh_), nPriv("~")
    {
        //Initialization


        nPriv.param<std::string>("detector_type", detectorType, "full");

        stringstream ss;
        ss << ros::package::getPath("pedestrian_detector");
        person_detector = new pedestrianDetector(conf_pedestrians, conf_heads, detectorType, ss.str());
        it = new image_transport::ImageTransport(nh);

        //Advertise
        image_pub = it->advertise("image_out", 1);
        detectionPublisher = nh.advertise<pedestrian_detector::DetectionList>("detections", 1);

        //Subscribe to vizzy's left camera
        //Change this later
        image_sub = it->subscribe("/vizzy/l_camera/image_rect_color", 1, &PedDetector::imageCb, this);
        //image_sub = it->subscribe("/vizzy/l_camera/image_raw", 1, &PedDetector::imageCb, this);
        //image_sub = it->subscribe("image_in", 1, &PedDetector::imageCb, this);
    }

    ~PedDetector()
    {
        delete person_detector;
        delete it;
    }


};


int main(int argc, char** argv)
{


    ros::init(argc, argv, "pedestrianDetector");

    ros::NodeHandle n;


    //Get package path. This way we dont need to worry about running the node in the folder of the configuration files
    stringstream ss;
    ss << ros::package::getPath("pedestrian_detector");
    ss << "/configuration.xml";

    stringstream ss2;
    ss2 << ros::package::getPath("pedestrian_detector");
    ss2 << "/configurationheadandshoulders.xml";

    stringstream ss3;
    ss3 << ros::package::getPath("pedestrian_detector");
    ss3 << "/partMasks.yaml";

    FileStorage fs(ss3.str(), FileStorage::READ);

    std::vector<int> headVect;
    std::vector<int> torsoVect;
    std::vector<int> legsVect;
    std::vector<int> feetVect;

    fs["headMask"] >> headVect;
    fs["torsoMask"] >> torsoVect;
    fs["legsMask"] >> legsVect;
    fs["feetMask"] >> feetVect;

    cv::Rect headMask(headVect[0], headVect[1], headVect[2], headVect[3]);
    cv::Rect torsoMask(torsoVect[0], torsoVect[1], torsoVect[2], torsoVect[3]);
    cv::Rect legsMask(legsVect[0], legsVect[1], legsVect[2], legsVect[3]);
    cv::Rect feetMask(feetVect[0], feetVect[1], feetVect[2], feetVect[3]);

    fs.release();

    partMasks.push_back(headMask);
    partMasks.push_back(torsoMask);
    partMasks.push_back(legsMask);
    partMasks.push_back(feetMask);


    PedDetector detector(n, ss.str(), ss2.str());

    ros::spin();
    //    myfile.close();
    return 0;
}
