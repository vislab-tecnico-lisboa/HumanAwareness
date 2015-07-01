#include "../include/tracker/personMotionModel.hpp"
#include <algorithm>
#include <vector>

/*
* Some motion models are implemented here in order to try to
* get better position estimates, based on the velocity and
* previous position of the person.
*
* TODO: Kalman filtering and motion prediction?
*/

void PersonMotion::updateVelocityArray(Point3d detectedPosition)
{
    for(int i=0; i < 5; i++)
        velocity[5-i] = velocity[5-(i+1)];

    if(detectedPosition.x != -1000 && detectedPosition.y != -1000)
    {
      velocity[0].x = (detectedPosition.x - position.x)/delta_t;
      velocity[0].y = (detectedPosition.y - position.y)/delta_t;
    }
     //If there is no suitabled detection we don't update it
    //This must be better thinked...
}


Point2d PersonMotion::getPositionEstimate()
{

    //For now just return the last detection...
    return position;

}


void PersonMotion::updateModel(Point3d detectedPosition)
{

 //Maybe applying a median filter to the velocity gives good predictive results

  //updateVelocityArray(detectedPosition);

  position.x = detectedPosition.x;
  position.y = detectedPosition.y;


  for(int i=0; i < 4; i++)
      positionHistory[4-i] = positionHistory[4-(i+1)];

  positionHistory[0] = position;

  //velocity = median...


}

PersonMotion::PersonMotion(Point3d detectedPosition)
{

    for(int i=0; i < 5; i++)
        velocity[i] = Point2d(0, 0);

  position.x = detectedPosition.x;
  position.y = detectedPosition.y;

  for(int i=0; i < 5; i++)
  {
      positionHistory[i].x =-1000;
      positionHistory[i].y = -1000;
  }

}

Point3d PersonMotion::getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point2d estimation)
{
    Point3d nearest(-1000, -1000, 0);
    Point3d estimation3d(estimation.x, estimation.y, 0);

    Mat est3d(estimation3d);

    if(coordsInBaseFrame.size() == 0)
        return nearest;

    double distance=1000000;

    for(vector<cv::Point3d>::iterator it = coordsInBaseFrame.begin(); it != coordsInBaseFrame.end(); it++)
      {

        double dist = norm(est3d, Mat((*it)), NORM_L2);
        if(dist < distance && dist < 2)
        {
            distance = dist;
            nearest = (*it);
        }

      }
    return nearest;
}

Point3d PersonMotion::medianFilter()
{
    //Optimize this later

    double x[5];
    double y[5];

    for(int i = 0; i<5; i++)
    {
        x[i] = positionHistory[i].x;
        y[i] = positionHistory[i].y;
    }

    vector<double> x_vect(x, x + sizeof(x)/sizeof(x[0]));
    vector<double> y_vect(y, y + sizeof(y)/sizeof(y[0]));

    sort(x_vect.begin(), x_vect.begin() + 5);
    sort(y_vect.begin(), y_vect.begin() + 5);


    Point3d medianPoint(x_vect.at(2), y_vect.at(2), 0);

    return medianPoint;


}

PersonMotion::~PersonMotion()
{

}
