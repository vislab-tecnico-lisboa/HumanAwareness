#ifndef CAMERAMODEL_HPP
#define CAMERAMODEL_HPP

#include <opencv/cv.hpp>
#include <stack>
#include <string>

using namespace std;

class cameraModel
{
  private:
  cv::Mat K_;
  cv::Mat distCoefs_;
  cv::Mat projectionMat_;
  cv::Mat pose;


  public:
  cv::Mat invertedK;

  cameraModel(string configFile);
  std::vector<cv::Point3d> calculatePointsOnWorldFrame(cv::Mat imagePoints, cv::Mat worldLinkToCamera);
  std::vector<cv::Point3d> calculatePointsOnWorldFrameWithoutHomography(vector<cv::Rect_<int> >* rects, cv::Mat baseLinkToWorld);
  cv::Mat getK();
  cv::Mat getDistCoefs();
  cv::Mat getProjectionMat();



};

#endif // CAMERAMODEL_HPP
