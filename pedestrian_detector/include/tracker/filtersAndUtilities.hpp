#ifndef FILTERSANDUTILITIES_H
#define FILTERSANDUTILITIES_H
#include <opencv2/opencv.hpp>
//Custom messages
#include <pedestrian_detector/DetectionList.h>
#include <pedestrian_detector/BoundingBox.h>
#include "../include/tracker/cameraModel.hpp"

using namespace cv;

double getZ(Point2d center, Point2d worldXY, Mat mapToCameraTransform, cameraModel *cameramodel);

class DetectionFilter
{

private:
    double maximum_person_height;
    double minimum_person_height;
    cameraModel *cameramodel;

public:
    DetectionFilter(float maximum_person_height, float minimum_person_height, cameraModel *cameramodel);
    void filterDetectionsByPersonSize(std::vector<cv::Point3d> &coordsInBaseFrame, vector<cv::Rect_<int> > rects, Mat mapToCameraTransform);
};



#endif // FILTERSANDUTILITIES_H


