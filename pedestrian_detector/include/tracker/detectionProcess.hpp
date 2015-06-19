#ifndef DETECTIONPROCESS_H
#define DETECTIONPROCESS_H
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stack>


using namespace cv;

int getArea(Rect_<int> detection);
Point2d getCenter(Rect_<int> detection);
Point2d getFeet(cv::Rect_<int> detection);

int getPersonDistance(cv::Rect_<int> detection);

class simpleFilter
{
  public:
  std::vector<Rect_<int> > lastDetections;

  std::vector<Rect_<int> > filter(std::vector<Rect_<int> >* detection);

};



#endif // DETECTIONPROCESS_H
