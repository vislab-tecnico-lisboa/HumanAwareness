#ifndef PERSONMOTIONMODEL_HPP
#define PERSONMOTIONMODEL_HPP

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class PersonMotion
{

    private:

    //Sampling time
    int delta_t;

    //Five last velocities (Fast Five. lol)
    Point2d velocity[5];

    Point2d filteredVelocity;
    Point2d position;


    void updateVelocityArray(Point3d detectedPosition);

    public:
    Point2d positionHistory[5];
    PersonMotion(Point3d detectedPosition);
    Point3d medianFilter();
    ~PersonMotion();
    Point2d getPositionEstimate();
    void updateModel(Point3d detectedPosition);
    Point3d getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point2d estimation);

};


#endif // PERSONMOTIONMODEL_HPP
