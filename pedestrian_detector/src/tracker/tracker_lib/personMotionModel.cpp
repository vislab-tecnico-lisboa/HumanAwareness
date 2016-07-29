#include "../include/tracker/personMotionModel.hpp"
#include <algorithm>
#include <vector>
#include <math.h>
#include "../include/HungarianFunctions.hpp"
#include "../include/tracker/cameraModel.hpp"

Mat PersonModel::getBvtHistogram()
{
    return bvtHistogram;
}

double PersonModel::getScoreForAssociation(double height, Point3d detectedPosition, Mat detectionColorHist, Mat trackerColorHist)
{
    //pdf of the height
    //double innovation = heightP+heightR;

    double p_colors = compareHist(detectionColorHist, trackerColorHist, CV_COMP_BHATTACHARYYA);
    //double p_height = (1.0/(sqrt(2*CV_PI)*sqrt(innovation)))*exp(-pow((height-personHeight), 2)/(2*innovation));


    //Get the pdfs from the mmae
    Mat measurement = Mat(2, 1, CV_64F);
    measurement.at<double>(0, 0) = detectedPosition.x;
    measurement.at<double>(1, 0) = detectedPosition.y;
    /*
    double density = 0;
    int l = 0;
    double p_models = 0;*/
    /*

    for(std::vector<KalmanFilter>::iterator it = mmaeEstimator->filterBank.begin(); it != mmaeEstimator->filterBank.end(); it++, l++)
    {
        Mat residue = measurement-mmaeEstimator->filterBank.at(l).measurementMatrix*mmaeEstimator->filterBank.at(l).statePre;
        residue.convertTo(residue, CV_64F);
        Mat trResidue;
        transpose(residue, trResidue);
        Mat q = trResidue*mmaeEstimator->invAk.at(l)*residue;
        q.convertTo(q, CV_64F);
        Mat expTerm = ((-1.0/2.0))*q;
        cv::exp(expTerm, expTerm);

        CV_Assert(expTerm.rows == 1 && expTerm.cols == 1);

        if(expTerm.type() == CV_32F)
        {
          density = mmaeEstimator->betas.at(l)*static_cast<double>(expTerm.at<float>(0, 0));
        }
        else if(expTerm.type() == CV_64F)
        {
          density = mmaeEstimator->betas.at(l)*expTerm.at<double>(0, 0);
        }
        p_models += density*mmaeEstimator->probabilities.at(l);
    }*/

    //return (500-(log(p_colors)+log(p_height)+log(p_models)));
    //return 500-p_colors*p_height*(p_const_pos+p_const_vel+p_const_accel);
    return p_colors;
}



Point3d PersonModel::getPositionEstimate()
{

    Point3d estimate(mmaeEstimator->xMMAE.at<double>(mmaeEstimator->modelToxMMAEindexes.at(0).at(0), 0), mmaeEstimator->xMMAE.at<double>(mmaeEstimator->modelToxMMAEindexes.at(0).at(1), 0), personHeight);
    return estimate;
}

Mat PersonModel::getCovarianceOfMixture()
{
    return mmaeEstimator->stateCovMMAE;
}


void PersonModel::updateModel()
{

    ros::Time now = ros::Time::now();
    ros::Duration sampleTime = now - lastUpdate;

    delta_t = sampleTime.toSec();
    lastUpdate = now;


    for(int i=0; i < median_window-1; i++)
        positionHistory[median_window-1-i] = positionHistory[median_window-1-(i+1)];

    positionHistory[0] = position;


    position.x = -1000;
    position.y = -1000;
    position.z = 0.95;

    Mat measurement;
    if(positionHistory[0].x == -1000 || positionHistory[0].y == -1000)
    {
        measurement = Mat();
    }
    else
    {
        measurement = Mat(2, 1, CV_64F);
        measurement.at<double>(0, 0) = positionHistory[0].x;
        measurement.at<double>(1, 0) = positionHistory[0].y;
    }
    mmaeEstimator->correct(measurement);

    //Correct the person height.
    heightK = heightP/(heightP+heightR);
    personHeight += heightK*(positionHistory[0].z*2-personHeight);
    heightP = (1-heightK)*heightP;

}

PersonModel::PersonModel(Point3d detectedPosition, cv::Rect_<int> bb, int id, int median_window, Mat bvtHistogram, int method)
{    

    static const double samplingRate = 100.0;
    T = 1.0/samplingRate;
    delta_t = T;  //Initialize delta t for first predict

    //Constant position filter

    KalmanFilter constantPosition(2, 2, 0, CV_64F);

    //Phi
    constantPosition.transitionMatrix = (Mat_<double>(2, 2) << 1, 0, 0, 1);
    //H
    constantPosition.measurementMatrix = (Mat_<double>(2, 2) << 1, 0, 0, 1);
    //Q
    constantPosition.processNoiseCov = (Mat_<double>(2, 2) << pow(T, 2), 0, 0, pow(T, 2));  //Q*sigma
    constantPosition.processNoiseCov  = constantPosition.processNoiseCov*1;
    //R
    constantPosition.measurementNoiseCov = (Mat_<double>(2, 2) << 1, 0, 0, 1);
    constantPosition.measurementNoiseCov = constantPosition.measurementNoiseCov*0.05; //R*sigma

    //P0
    constantPosition.errorCovPost = Mat::eye(2, 2, CV_64F)*0.5;

    //Constant velocity filter

    KalmanFilter constantVelocity(4, 2, 0, CV_64F);

    //Phi
    constantVelocity.transitionMatrix = (Mat_<double>(4, 4) << 1, 0, T, 0, 0, 1, 0, T, 0, 0, 1, 0, 0, 0, 0, 1);
    //H
    constantVelocity.measurementMatrix = (Mat_<double>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    //Q1
    constantVelocity.processNoiseCov = (Mat_<double>(4, 4) << pow(T,4)/4, 0, pow(T,3)/2, 0, 0, pow(T,4)/4, 0, pow(T,3)/2, pow(T,3)/2, 0, pow(T,2), 0, 0, pow(T,3)/2, 0, pow(T,2));
    constantVelocity.processNoiseCov  = constantVelocity.processNoiseCov*1;  //Q*sigma
    //R
    constantVelocity.measurementNoiseCov = (Mat_<double>(2, 2) << 1, 0, 0, 1);
    constantVelocity.measurementNoiseCov = constantVelocity.measurementNoiseCov*0.05; //R*sigma

    //P0
    constantVelocity.errorCovPost = Mat::eye(4, 4, CV_64F)*0.5;

    //Constant acceleration filter

    KalmanFilter constantAcceleration(6, 2, 0, CV_64F);

    //Phi
    constantAcceleration.transitionMatrix = (Mat_<double>(6, 6) << 1, 0, T, 0, pow(T, 2)/2, 0, 0, 1, 0, T, 0, pow(T, 2)/2, 0, 0, 1, 0, T, 0, 0, 0, 0, 1, 0, T, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 );
    //H
    constantAcceleration.measurementMatrix = (Mat_<double>(2, 6) << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);
    //Q
    constantAcceleration.processNoiseCov = (Mat_<double>(6, 6) << pow(T, 5)/20, 0, pow(T, 4)/8, 0, pow(T, 3)/6, 0, 0, pow(T, 5)/20, 0, pow(T, 4)/8, 0, pow(T, 3)/6, pow(T, 4)/8, 0, pow(T, 3)/6, 0, pow(T, 2)/2, 0, 0, pow(T, 4)/8, 0, pow(T, 3)/6, 0, pow(T, 2)/2, pow(T, 3)/6, 0, pow(T, 2)/2, 0, T, 0, 0, pow(T, 3)/6, 0, pow(T, 2)/2, 0, T);
    constantAcceleration.processNoiseCov  = constantAcceleration.processNoiseCov*1;  //Q*sigma
    //R
    constantAcceleration.measurementNoiseCov = (Mat_<double>(2, 2) << 1, 0, 0, 1);
    constantAcceleration.measurementNoiseCov = constantAcceleration.measurementNoiseCov*0.05; //R*sigma

    //P0
    constantAcceleration.errorCovPost = Mat::eye(6, 6, CV_64F)*0.5;

    // Put them all in the bank
    std::vector<KalmanFilter> kalmanBank;
    kalmanBank.push_back(constantPosition);
    kalmanBank.push_back(constantVelocity);
    kalmanBank.push_back(constantAcceleration);

    // Link each of these filters states to the ponderated state

    //State [i] from models corresponds to State indexes[i] in xMMAEx

    /*      With 3 models
 *      int indexes1[] = {0, 3};
        int indexes2[] = {0, 1, 3, 4};
        int indexes3[] = {0, 1, 2, 3, 4, 5};*/

    int indexes1[] = {0, 1};
    int indexes2[] = {0, 1, 2, 3};
    int indexes3[] = {0, 1, 2, 3, 4, 5};

    std::vector<int> indexes1_(indexes1, indexes1+sizeof(indexes1)/sizeof(int));
    std::vector<int> indexes2_(indexes2, indexes2+sizeof(indexes2)/sizeof(int));
    std::vector<int> indexes3_(indexes3, indexes3+sizeof(indexes3)/sizeof(int));

    std::vector<std::vector<int> > indexList;
    indexList.push_back(indexes1_);
    indexList.push_back(indexes2_);
    indexList.push_back(indexes3_);

    //Guess the initial states - a good guess for the position is the measurement we just got!

    //        For 3 models
    double states[] = {detectedPosition.x, detectedPosition.y, 0, 0, 0, 0};
    //        double states[] = {0, 0, 0, 0, 0, 0};
    Mat initialStates = Mat(6, 1, CV_64F, states).clone();

    //        double states[] = {detectedPosition.x, 0, detectedPosition.y, 0};
    //        Mat initialStates = Mat(4, 1, CV_64F, states).clone();

    //Build the Bank!
    mmaeEstimator = new MMAEFilterBank(kalmanBank, indexList, false, false, initialStates, CV_64F);

    this->median_window = median_window;
    toBeDeleted = false;

    position = detectedPosition;

    /*Kalman for position*/
    personHeight = detectedPosition.z*2;
    heightP = 0.1;
    heightQ = 0;
    heightR = 0.0146;

    /*Im not creating a class for 5 variables...*/


    this->bvtHistogram = bvtHistogram;


    rect = bb;

    positionHistory[0] = detectedPosition;


    for(int i=1; i < median_window; i++)
    {
        positionHistory[i].x =-1000;
        positionHistory[i].y = -1000;
        positionHistory[i].z = 0.95;
    }


    lockedOnce = false;

    this->id = id;
    noDetection = 0;

    lastUpdate = ros::Time::now();

    deadReckoning = false;
}

Point3d PersonModel::getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point3d estimation)
{
    Point3d nearest(-1000, -1000, 0);
    Point3d estimation3d(estimation.x, estimation.y, 0);

    if(coordsInBaseFrame.size() == 0)
        return nearest;

    double distance=1000000;



    for(vector<cv::Point3d>::iterator it = coordsInBaseFrame.begin(); it != coordsInBaseFrame.end(); it++)
    {
        //It's dumb and slow to create matrix headers just to wrap 2 points...
        //I'm leaving this commented just to remember that!
        // double dist = norm(est3d, Mat((*it)), NORM_L2);

        double dist = norm(estimation3d-(*it));
        if(dist < distance && dist < 0.5)
        {
            distance = dist;
            nearest = (*it);
        }

    }
    return nearest;
}

Point3d PersonModel::medianFilter()
{
    //Optimize this later

    double x[median_window];
    double y[median_window];
    double z[median_window];

    for(int i = 0; i<median_window; i++)
    {
        x[i] = positionHistory[i].x;
        y[i] = positionHistory[i].y;
        z[i] = positionHistory[i].z;
    }

    vector<double> x_vect(x, x + sizeof(x)/sizeof(x[0]));
    vector<double> y_vect(y, y + sizeof(y)/sizeof(y[0]));
    vector<double> z_vect(z, z + sizeof(z)/sizeof(y[0]));

    std::sort(x_vect.begin(), x_vect.begin() + median_window);
    std::sort(y_vect.begin(), y_vect.begin() + median_window);
    std::sort(z_vect.begin(), z_vect.begin() + median_window);

    Point3d medianPoint(x_vect.at((int) (median_window/2)), y_vect.at((int) (median_window/2)), z_vect.at((int) (median_window/2)));

    return medianPoint;

}


PersonList::PersonList(int median_window, int numberOfFramesBeforeDestruction, int numberOfFramesBeforeDestructionLocked, double associatingDistance, int method)
{
    //This will never get reseted. That will make sure that we have a new id for every new detection
    nPersons = 0;
    this->numberOfFramesBeforeDestruction = numberOfFramesBeforeDestruction;
    this->numberOfFramesBeforeDestructionLocked = numberOfFramesBeforeDestructionLocked;
    this-> associatingDistance = associatingDistance;
    this->median_window = median_window;

}

PersonList::~PersonList()
{
    for(std::vector<PersonModel>::iterator it = personList.begin(); it!=personList.end(); it++)
    {
        if(it->mmaeEstimator != NULL)
            delete it->mmaeEstimator;
    }
}

void PersonList::updateList()
{
    //TODO

    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end();)
    {

        //Predict the right ammount of times to simulate a constant sampling period
        int times = it->delta_t/it->T;
        for(int lol = 0; lol < times; lol++)
            it->mmaeEstimator->predict();

        //Predict the height covariance P_minus. No need to predict the size. We assume
        //its a constant model
        it->heightP += it->heightQ; //

        if((*it).position.x != -1000 && (*it).position.y != -1000)
        {
            (*it).noDetection = 0;
            (*it).deadReckoning = false;
        }
        else
        {
            (*it).noDetection++;
            //Dead reckoning
            (*it).deadReckoning = true;
        }
        (*it).updateModel();

        if(((*it).noDetection > numberOfFramesBeforeDestruction && (*it).lockedOnce==false) || ((*it).noDetection > numberOfFramesBeforeDestructionLocked && (*it).lockedOnce==true))
        {
            it->toBeDeleted = true;
        }

        it++;
    }

}

void PersonList::addPerson(Point3d pos, cv::Rect_<int> rect, Mat bvtHistogram)
{
    PersonModel person(pos, rect, nPersons, median_window, bvtHistogram);
    personList.push_back(person);
    nPersons++;

}

void PersonList::associateData(vector<Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects, vector<Mat> colorFeaturesList)
{
    //Create distance matrix.
    //Rows represent the detections and columns represent the trackers


    int nTrackers = personList.size();
    int nDetections = coordsInBaseFrame.size();

    if(nTrackers > 0 && nDetections > 0)
    {

        double *distMatrixIn = (double*) malloc(sizeof(double)*nTrackers*nDetections);
        double *assignment = (double*) malloc(sizeof(double)*nDetections);
        double *cost = (double*) malloc(sizeof(double)*nDetections);

        //Fill the matrix
        /*  _           _
    *  |  l1c1 l1c2  |
    *  |  l2c1 l2c2  |
    *  |_ l3c1 l3c2 _|
    *
    *  [l1c1, l2c1, l3c1, l1c2,l2c2,l3c2]
    */

        int row = 0;

        //Each detection -> a row
        for(vector<Point3d>::iterator itrow = coordsInBaseFrame.begin(); itrow != coordsInBaseFrame.end(); itrow++, row++)
        {
            //Each tracker -> a column
            int col = 0;
            for(vector<PersonModel>::iterator itcolumn = personList.begin(); itcolumn != personList.end(); itcolumn++, col++)
            {

                //Calculate the distance

                Point3d trackerPos2d = (*itcolumn).getPositionEstimate();
                Mat trackerColorHist = (*itcolumn).getBvtHistogram();
                Mat detectionColorHist = colorFeaturesList.at(row);


                detectionColorHist.convertTo(detectionColorHist, CV_32F);
                trackerColorHist.convertTo(trackerColorHist, CV_32F);

                Point3d trackerPos;

                trackerPos.x = trackerPos2d.x;
                trackerPos.y = trackerPos2d.y;
                trackerPos.z = 0.0;

                Point3d detectionPos = (*itrow);

                double height = detectionPos.z*2.0;

                detectionPos.z = 0.0;

                double dist = norm(trackerPos-detectionPos); // usar distancia de mahalanobis

                double colorDist;

                colorDist = compareHist(detectionColorHist, trackerColorHist, CV_COMP_BHATTACHARYYA);
                if(dist < 3.0)
                {
                    if(colorDist > 0.7)
                        distMatrixIn[row+col*nDetections] = 1000; //Colors are too different. It cant be the same person...
                    else
                        distMatrixIn[row+col*nDetections] = itcolumn->getScoreForAssociation(height, detectionPos, detectionColorHist, trackerColorHist);
                }
                else
                {
                    double best = 1000;
                    for(int i = 1; i<median_window; i++)
                    {
                        Point3d hist((*itcolumn).positionHistory[i].x, (*itcolumn).positionHistory[i].y, 0);
                        double distHist = norm(hist-detectionPos);
                        if(distHist < 1.8)
                            if(distHist < best)
                                best = distHist;
                    }

                    distMatrixIn[row+col*nDetections] = itcolumn->getScoreForAssociation(height, detectionPos, detectionColorHist, trackerColorHist);
                }
            }
        }

        assignmentoptimal(assignment, cost, distMatrixIn, nDetections, nTrackers);

        //assignment vector positions represents the detections and the value in each position represents the assigned tracker
        //if there is no possible association, then the value is -1
        //cost vector doesn't mean anything at this point...

        //for each detection we update it's tracker with the position

        for(int i=0; i<nDetections; i++)
        {

            //If there is an associated tracker and the distance is less than 2 meters we update the position
            if(assignment[i] != -1)
            {

                Point3d detPos(coordsInBaseFrame.at(i).x, coordsInBaseFrame.at(i).y, 0);
                Point3d trackPos = personList.at(assignment[i]).getPositionEstimate();
                Mat trackerColorHist = personList.at(assignment[i]).getBvtHistogram();
                Mat detectionColorHist = colorFeaturesList.at(i);
                double colorDist = compareHist(detectionColorHist, trackerColorHist, CV_COMP_BHATTACHARYYA);
                trackPos.z = 0.0;

                if(norm(detPos-trackPos) < 2.0 && colorDist < 0.7)
                {
                    personList.at(assignment[i]).position.x = coordsInBaseFrame.at(i).x;
                    personList.at(assignment[i]).position.y = coordsInBaseFrame.at(i).y;
                    personList.at(assignment[i]).position.z = coordsInBaseFrame.at(i).z;


                    //Bounding boxes...
                    personList.at(assignment[i]).rect = rects.at(i);

                    //Color histograms I'm not going to update it now lol
                    personList.at(assignment[i]).bvtHistogram = personList.at(assignment[i]).bvtHistogram*0.8+0.2*colorFeaturesList.at(i);
                }
                else
                {

                    //If there isn't, create a new tracker for each one - IF THERE IS NO OTHER TRACKER IN A 1.5m radius
                    bool existsInRadius = false;

                    Point3d coords = coordsInBaseFrame.at(i);
                    coords.z = 0.0;

                    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
                    {
                        Point3d posEstimate = it->getPositionEstimate();

                        Point3d testPoint(posEstimate.x, posEstimate.y, 0);

                        if(norm(coords-testPoint) < 1.5)
                        {
                            ROS_DEBUG("NOT CREATING: Exists in radius");
                            existsInRadius = true;
                            break;
                        }
                    }
                    if(!existsInRadius)
                    {
                        ROS_DEBUG("Creating without -1");
                        addPerson(coordsInBaseFrame.at(i), rects.at(i), colorFeaturesList.at(i));
                    }
                }
            }
            else
            {
                //If there isn't, create a new tracker for each one - IF THERE IS NO OTHER TRACKER IN A 1.5m radius
                bool existsInRadius = false;

                Point3d coords = coordsInBaseFrame.at(i);
                coords.z = 0.0;

                for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
                {
                    Point3d posEstimate = it->getPositionEstimate();

                    Point3d testPoint(posEstimate.x, posEstimate.y, 0.0);

                    if(norm(coords-testPoint) < associatingDistance)
                    {
                        ROS_DEBUG("NOT CREATING: Exists in radius");
                        existsInRadius = true;
                        break;
                    }
                }
                if(!existsInRadius)
                {
                    ROS_DEBUG("Creating");
                    addPerson(coordsInBaseFrame.at(i), rects.at(i), colorFeaturesList.at(i));
                }
            }
        }


        //free the memory!

        free(distMatrixIn);
        free(assignment);
        free(cost);

    }
    else
    {
        //We create a tracker for each detection, and if there are none, we do nothing! - can't create 2 trackers in the same 1.5m radius
        for(int i=0; i<nDetections; i++)
        {
            bool existsInRadius = false;

            Point3d coords = coordsInBaseFrame.at(i);
            coords.z = 0.0;

            for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
            {
                Point3d testPoint((*it).positionHistory[0].x, (*it).positionHistory[0].y, 0.0);
                if(norm(coords-testPoint) < associatingDistance)
                {
                    ROS_DEBUG("NOT CREATING: Existis in radius");
                    existsInRadius = true;
                    break;
                }
            }
            if(!existsInRadius)
            {
                ROS_DEBUG("Creating");
                addPerson(coordsInBaseFrame.at(i), rects.at(i), colorFeaturesList.at(i));
            }
        }

    }
    //for each associated tracker we update the detection. For everyone else there is Mastercard. Just kidding... trackers
    //wich have no detections associated will be updated with a -1000, -1000 detection

    updateList();

}

std::vector<PersonModel> PersonList::getValidTrackerPosition()
{

    std::vector<PersonModel> validTrackers;

    for(std::vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
    {
        Point3d estimate = it->getPositionEstimate();
        if(estimate.x != -1000 && estimate.y != -1000)
        {
            validTrackers.push_back(*it);
        }

    }

    return validTrackers;

}

std::vector<int> PersonList::trackletKiller()
{

    std::vector<int> deletedTracklets;

    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end();)
    {
        if(it->toBeDeleted)
        {
            deletedTracklets.push_back(it->id);
            it = personList.erase(it);
        }
        else
        {
            it++;
        }
    }

    return deletedTracklets;
}

cv::Mat PersonList::plotReprojectionAndProbabilities(int targetId, cv::Mat baseFootprintToCameraTransform, CameraModel *cameramodel, cv::Mat lastImage)
{
    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
    {

        //Get feet position
        Point3d head = it->getPositionEstimate();

        Point3d feet = head;
        feet.z = 0;

        //Project them into the image
        Point2d feetOnImage;
        Point2d headOnImage;

        /*Code*/
        Mat headMat = Mat::ones(4, 1, CV_32F);
        headMat.at<float>(0,0) = head.x;
        headMat.at<float>(1,0) = head.y;
        headMat.at<float>(2,0) = head.z;

        Mat feetMat = Mat::ones(4, 1, CV_32F);
        feetMat.at<float>(0,0) = feet.x;
        feetMat.at<float>(1,0) = feet.y;
        feetMat.at<float>(2,0) = feet.z;

        Mat intrinsics = cameramodel->getK();
        intrinsics.convertTo(intrinsics, CV_32F);

        Mat RtMat = baseFootprintToCameraTransform(Range(0, 3), Range(0, 4));
        RtMat.convertTo(RtMat, CV_32F);

        Mat feetOnImageMat = intrinsics*RtMat*feetMat;
        Mat headOnImageMat = intrinsics*RtMat*headMat;

        feetOnImage.x = feetOnImageMat.at<float>(0,0)/feetOnImageMat.at<float>(2,0);
        feetOnImage.y = feetOnImageMat.at<float>(1,0)/feetOnImageMat.at<float>(2,0);

        headOnImage.x = headOnImageMat.at<float>(0,0)/headOnImageMat.at<float>(2,0);
        headOnImage.y = headOnImageMat.at<float>(1,0)/headOnImageMat.at<float>(2,0);

        //Draw the rectangle from the point

        int h = feetOnImage.y-headOnImage.y;
        int width = h*52/128;

        int topLeftX = headOnImage.x - width/2;
        int topLeftY = headOnImage.y;

        cv::Rect_<int> trackedBB(topLeftX, topLeftY, width, h);

        if(h > 0)
        {

            //Bar plot of probabilities

            int step = trackedBB.width/3;
            int maxBarSize = trackedBB.height*4/5;
            const static Scalar cores[3] = {Scalar(24, 54, 231), Scalar(239, 232, 31), Scalar(255, 0, 0)};

            int ind = 0;
            Point2d barBase(trackedBB.tl().x, trackedBB.br().y); //Bottom left corner of the bb
            for(std::vector<double>::iterator pr = it->mmaeEstimator->probabilities.begin(); pr != it->mmaeEstimator->probabilities.end(); pr++)
            {
                Point2d tl = barBase-Point2d(0, maxBarSize*(*pr));
                rectangle(lastImage, tl, barBase + Point2d(step, 0), cores[ind], CV_FILLED);

                if(ind < 2)
                    ind++;
                else
                    ind = 0;

                barBase = barBase + Point2d(step, 0);
            }

            ostringstream convert;

            if(it->id == targetId)
            {
                rectangle(lastImage, trackedBB, Scalar(0, 0, 255), 2);
                convert << "id: " << it->id << " | H = " << head.z;

            }else{
                rectangle(lastImage, trackedBB, Scalar(0, 255, 0), 2);
                convert << "id: " << it->id;
            }

            putText(lastImage, convert.str(), trackedBB.tl(), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar_<int>(255,0,0), 2);

        }
    }

    return lastImage;
}

