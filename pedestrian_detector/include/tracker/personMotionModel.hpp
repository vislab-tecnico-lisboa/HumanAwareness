#ifndef PERSONMOTIONMODEL_HPP
#define PERSONMOTIONMODEL_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>


using namespace cv;
using namespace std;

class PersonModel
{

    private:

    //Sampling time - not used yet
    ros::Time lastUpdate;

    double delta_t;



    //Five last velocities (Fast Five. lol) - not used yet
    Point2d velocity[25];
    Point2d filteredVelocity;
    void updateVelocityArray(Point2d detectedPosition);

    public:

    int id;

    Point2d bbCenter;

    bool lockedOnce;
    bool deadReckoning;
    //Increment this each time there is no detection and reset it when there is a detection
    //If this counter equals 5 we destroy this tracker.
    int noDetection;

    Point2d position;

    Point2d positionHistory[5];
    cv::Rect_<int> rectHistory[5];
    cv::Rect rect;

    PersonModel(Point3d detectedPosition, cv::Rect_<int> bb, int id);
    Point3d medianFilter();
    ~PersonModel();
    Point2d getPositionEstimate();
    void updateModel();
    Point3d getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point2d estimation);
    Point2d velocityMedianFilter();
};

class PersonList
{

public:

    int nPersons;

    void associateData(vector<cv::Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects);
    void addPerson(Point3d pos, cv::Rect_<int> rect);
    std::vector<PersonModel> personList;
    void updateList();

    PersonList();

    //Returns a vector containing positions associated to each tracker, that are valid after the median filter
    std::vector<PersonModel> getValidTrackerPosition();


};

#endif // PERSONMOTIONMODEL_HPP
