#ifndef FILTERSANDUTILITIES_H
#define FILTERSANDUTILITIES_H
#include <opencv2/opencv.hpp>
//Custom messages
#include "../include/tracker/cameraModel.hpp"

using namespace cv;

double getZ(Point2d center, Point2d worldXY, Mat mapToCameraTransform, CameraModel *cameramodel);

class DetectionFilter
{

private:
    double maximum_person_height;
    double minimum_person_height;
    CameraModel *cameramodel;

public:
    DetectionFilter(float maximum_person_height, float minimum_person_height, CameraModel *cameramodel);
    void filterDetectionsByPersonSize(std::vector<cv::Point3d> &coordsInBaseFrame, vector<cv::Rect_<int> > &rects, Mat mapToCameraTransform, vector<Mat> &colorFeaturesList, vector<double> &lambdas);
};



#endif // FILTERSANDUTILITIES_H


