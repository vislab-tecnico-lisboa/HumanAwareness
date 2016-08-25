#ifndef PERSONMOTIONMODEL_HPP
#define PERSONMOTIONMODEL_HPP

#define MMAETRACKING 1
#define MEDIANTRACKING 0

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include "../include/tracker/mmae.hpp"



using namespace cv;
using namespace std;

class PersonModel
{

    private:
    int median_window;

    Rect_<int> projectBoundingBox();


    public:
    double const_pos_var;
    double const_vel_var;
    double const_accel_var;

    int id;
    double metric_weight;
    MMAEFilterBank *mmaeEstimator;
    //KALMAN de uma variavel ahahahahahaha
    double personHeight;
    double heightP;
    double heightQ;
    double heightR;
    double heightK;
    //************************************
    double delta_t;
    double T;
    bool toBeDeleted;

    Point2d bbCenter;

    Mat bvtHistogram;

    bool lockedOnce;
    bool deadReckoning;
    //Increment this each time there is no detection and reset it when there is a detection
    //If this counter equals 5 we destroy this tracker.
    int noDetection;

    Point3d position;
    Mat getBvtHistogram();

    Point3d positionHistory[100];
    cv::Rect_<int> rectHistory[5];
    cv::Rect rect;

    PersonModel(Point3d detectedPosition, cv::Rect_<int> bb, int id, int median_window, Mat bvtHistogram);

    Point3d medianFilter();
    Point3d getPositionEstimate();
    Mat getCovarianceOfMixture();
    void updateModel();
    Point3d getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point3d estimation);
    double getScoreForAssociation(double height, Point3d detectedPosition, Mat detCov, Mat detectionColorHist, Mat trackerColorHist);
};

class PersonList
{

public:

    int nPersons;
    int median_window;
    int numberOfFramesBeforeDestruction;
    int numberOfFramesBeforeDestructionLocked;
    double validation_gate;
    double metric_weight;
    double creation_threshold;
    double recognition_threshold;
    double c_learning_rate;
    double const_pos_var;
    double const_vel_var;
    double const_accel_var;
    void associateData(vector<cv::Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects, vector<Mat> colorFeaturesList, vector<Mat> means, vector<Mat> covariances);
    void addPerson(Point3d pos, cv::Rect_<int> rect, Mat bvtHistogram);
    std::vector<PersonModel> personList;
    std::deque<PersonModel> holdList;   //For Re-ID purposes
    void predictList();
    void updateList();
    void updateDeltaT(double delta_t);

    PersonList(int median_window, int numberOfFramesBeforeDestruction, int numberOfFramesBeforeDestructionLocked, double creation_threshold, double validation_gate, double metric_weight, double recognition_threshold, double c_learning_rate, double const_pos_var, double const_vel_var, double const_accel_var);
    ~PersonList();
    //Returns a vector containing positions associated to each tracker, that are valid after the median filter
    std::vector<PersonModel> getValidTrackerPosition();
    //Returns a list of deleted tracklets
    std::vector<int> trackletKiller();
    cv::Mat plotReprojectionAndProbabilities(int targetId, cv::Mat baseFootprintToCameraTransform, cv::Mat K, cv::Mat lastImage);



};


#endif // PERSONMOTIONMODEL_HPP

