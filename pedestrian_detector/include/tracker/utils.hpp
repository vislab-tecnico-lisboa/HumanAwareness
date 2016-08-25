#ifndef UTILS_HPP
#define UTILS_HPP
#include <opencv/cv.h>
#include <iostream>
using namespace cv;

double mahalanobisDist(cv::Mat& x, cv::Mat& u, cv::Mat& S);
double bhattacharyyaDist(cv::Mat& u1, cv::Mat& S1, cv::Mat& u2, cv::Mat& S2);
double hellingerDist(cv::Mat& u1, cv::Mat& S1, cv::Mat& u2, cv::Mat& S2);
void computeMeasurementStatistics(Mat K, Mat transform, std::vector<double> lambdas, std::vector<cv::Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects,std::vector<cv::Mat> &meansArray, std::vector<cv::Mat> &covMatrices);

#endif // UTILS_HPP
