#include "../include/tracker/detectionProcess.hpp"
#include <algorithm>

using namespace cv;

//Computes the area of a rectangle. Might be usefull to find out the
//distance of pedestrians
int getArea(Rect_<int> detection)
{
  return detection.area();
}


//Gets the center of the detection
Point2d getCenter(Rect_<int> detection)
{
  int xc = detection.tl().x + detection.width/2;
  int yc = detection.tl().y + detection.height/2;

  Point2d center(xc, yc);
  return center;
}


//Gets the center of the bottom rectangle limit
Point2d getFeet(cv::Rect_<int> detection)
{
  int x = detection.tl().x+detection.width/2;
  int y = detection.br().y;
  Point2d feet(x, y);
  return feet;
}


//If we have the same detection in two consecutive frames, then we consider it valid
//I consider it the same detection if it inside a rectangle with 10 more pixels of
//on the width and height

std::vector<Rect_<int> > simpleFilter::filter(std::vector<Rect_<int> >* detection)
{
  std::vector<Rect_<int> > filteredList;

  std::vector<cv::Rect_<int> >::iterator it;
  std::vector<cv::Rect_<int> >::iterator it2;

  Point_<int> point(10,10);

  for(it = detection->begin(); it != detection->end(); it++)
    {
      for(it2=lastDetections.begin(); it2 != lastDetections.end(); it2++)
        {
          if((*it2).contains((*it).tl()+point) && (*it2).contains((*it).br()-point))
            filteredList.push_back((*it));
        }
    }

  lastDetections.clear();
  lastDetections = *detection;




  return filteredList;

}

//This function is bad... really bad...
int getPersonDistance(cv::Rect_<int> detection)
{
  int distance;

  distance = 318611.9274*pow(detection.height, -1.0096);

  return distance;
}

