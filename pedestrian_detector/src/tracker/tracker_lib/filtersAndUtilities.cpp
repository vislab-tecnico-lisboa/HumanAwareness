#include "../include/tracker/filtersAndUtilities.hpp"
#include <ros/ros.h>



//Compute the z coordinate in the world frame knowing both x and y

double getZ(Point2d center, Point2d worldXY, Mat mapToCameraTransform, cameraModel *cameramodel)
{
    Mat K = cameramodel->getK();
    Mat RT = mapToCameraTransform(Range(0,3), Range(0, 4));

    K.convertTo(K,CV_64FC1);
    RT.convertTo(RT, CV_64FC1);

    Mat P = K*RT;


    double z = worldXY.x*(center.x/center.y*P.at<double>(1,0)-P.at<double>(0,0))+worldXY.y*(center.x/center.y*P.at<double>(1,1)-P.at<double>(0,1))+center.x/center.y*P.at<double>(1,3)-P.at<double>(0,3);

    z = z/(P.at<double>(0,2)-center.x/center.y*P.at<double>(1,2));

    return z;
}


DetectionFilter::DetectionFilter(float maximum_person_height, float minimum_person_height, cameraModel *cameramodel)
{
    this->maximum_person_height = maximum_person_height;
    this->minimum_person_height = minimum_person_height;
    this->cameramodel = cameramodel;
}

//There will be detections that make no sense, so we can filter them by person size.
void  DetectionFilter::filterDetectionsByPersonSize(std::vector<cv::Point3d> &coordsInBaseFrame, vector<cv::Rect_<int> > rects, Mat mapToCameraTransform)
{

    vector<cv::Rect_<int> >::iterator itRect = rects.begin();

    for(vector<Point3d>::iterator itCoords = coordsInBaseFrame.begin(); itCoords != coordsInBaseFrame.end();)
    {
        cv::Point3d coordsOnXYPlane;
        coordsOnXYPlane = *itCoords;
        cv::Rect_<int> currentRect = *itRect;

        cv::Point2d topMiddle = currentRect.tl();  //Top left corner + width/2 on the x coordinate for it to be centered
        topMiddle = topMiddle + cv::Point2d(currentRect.width/2, 0);

        double z = getZ(topMiddle, Point2d(coordsOnXYPlane.x, coordsOnXYPlane.y), mapToCameraTransform, cameramodel);

        if(z > maximum_person_height || z < minimum_person_height)
        {
            //discard detection
            itRect = rects.erase(itRect);
            itCoords = coordsInBaseFrame.erase(itCoords);
        }
        else
        {
            itRect++;
            itCoords++;
        }
    }
}
