#include "../include/tracker/personMotionModel.hpp"
#include <algorithm>
#include <vector>
#include <math.h>
#include "../include/HungarianFunctions.hpp"
#include "../include/tracker/utils.hpp"

Mat PersonModel::getBvtHistogram()
{
    return bvtHistogram;
}

double PersonModel::getScoreForAssociation(double height, Point3d detectedPosition, Mat detCov, Mat detectionColorHist, Mat trackerColorHist)
{

    double p_colors = compareHist(detectionColorHist, trackerColorHist, CV_COMP_BHATTACHARYYA);


    Mat measurement = Mat(2, 1, CV_64F);
    measurement.at<double>(0, 0) = detectedPosition.x;
    measurement.at<double>(1, 0) = detectedPosition.y;

    Mat aux;
    this->getCovarianceOfMixture().copyTo(aux);
    Mat errorCovPost = aux(cv::Range(0,2),cv::Range(0,2));

    Point3d trackerPos2d = this->getPositionEstimate();

    Mat tracker2dPos = Mat::zeros(2, 1, CV_32F);
    tracker2dPos.at<float>(0,0) = trackerPos2d.x;
    tracker2dPos.at<float>(1,0) = trackerPos2d.y;


    //double dist = hellingerDist(measurement, detCov, tracker2dPos, errorCovPost);


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

    for(int i=0; i < median_window-1; i++)
        positionHistory[median_window-1-i] = positionHistory[median_window-1-(i+1)];

    positionHistory[0] = position;


    position.x = -1000;
    position.y = -1000;
    position.z = 0.95;

    Mat measurement;

    cv::Point3d filteredPoint;

    filteredPoint = medianFilter();

    if(filteredPoint.x == -1000 || filteredPoint.y == -1000)
    {
        measurement = Mat();
    }
    else
    {
        measurement = Mat(2, 1, CV_64F);
        measurement.at<double>(0, 0) = filteredPoint.x;
        measurement.at<double>(1, 0) = filteredPoint.y;
    }
    mmaeEstimator->correct(measurement);

    //Correct the person height.
    heightK = heightP/(heightP+heightR);
    personHeight += heightK*(positionHistory[0].z*2-personHeight);
    heightP = (1-heightK)*heightP;

}

PersonModel::PersonModel(Point3d detectedPosition, cv::Rect_<int> bb, int id, int median_window, Mat bvtHistogram)
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
    constantPosition.processNoiseCov  = constantPosition.processNoiseCov*const_pos_var;
    //R
    constantPosition.measurementNoiseCov = (Mat_<double>(2, 2) << 0.4421, 0.3476, 0.3476, 0.2747);
    constantPosition.measurementNoiseCov = constantPosition.measurementNoiseCov*1; //R*sigma

    //P0
    constantPosition.errorCovPost = Mat::eye(2, 2, CV_64F)*100;

    //Constant velocity filter

    KalmanFilter constantVelocity(4, 2, 0, CV_64F);

    //Phi
    constantVelocity.transitionMatrix = (Mat_<double>(4, 4) << 1, 0, T, 0, 0, 1, 0, T, 0, 0, 1, 0, 0, 0, 0, 1);
    //H
    constantVelocity.measurementMatrix = (Mat_<double>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
    //Q1
    constantVelocity.processNoiseCov = (Mat_<double>(4, 4) << pow(T,4)/4, 0, pow(T,3)/2, 0, 0, pow(T,4)/4, 0, pow(T,3)/2, pow(T,3)/2, 0, pow(T,2), 0, 0, pow(T,3)/2, 0, pow(T,2));
    constantVelocity.processNoiseCov  = constantVelocity.processNoiseCov*const_vel_var;  //Q*sigma
    //R
    constantVelocity.measurementNoiseCov = (Mat_<double>(2, 2) << 0.4421, 0.3476, 0.3476, 0.2747);
    constantVelocity.measurementNoiseCov = constantVelocity.measurementNoiseCov*1; //R*sigma

    //P0
    constantVelocity.errorCovPost = Mat::eye(4, 4, CV_64F)*100;

    //Constant acceleration filter

    KalmanFilter constantAcceleration(6, 2, 0, CV_64F);

    //Phi
    constantAcceleration.transitionMatrix = (Mat_<double>(6, 6) << 1, 0, T, 0, pow(T, 2)/2, 0, 0, 1, 0, T, 0, pow(T, 2)/2, 0, 0, 1, 0, T, 0, 0, 0, 0, 1, 0, T, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 );
    //H
    constantAcceleration.measurementMatrix = (Mat_<double>(2, 6) << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);
    //Q
    constantAcceleration.processNoiseCov = (Mat_<double>(6, 6) << pow(T, 5)/20, 0, pow(T, 4)/8, 0, pow(T, 3)/6, 0, 0, pow(T, 5)/20, 0, pow(T, 4)/8, 0, pow(T, 3)/6, pow(T, 4)/8, 0, pow(T, 3)/6, 0, pow(T, 2)/2, 0, 0, pow(T, 4)/8, 0, pow(T, 3)/6, 0, pow(T, 2)/2, pow(T, 3)/6, 0, pow(T, 2)/2, 0, T, 0, 0, pow(T, 3)/6, 0, pow(T, 2)/2, 0, T);
    constantAcceleration.processNoiseCov  = constantAcceleration.processNoiseCov*const_accel_var;  //Q*sigma
    //R
    constantAcceleration.measurementNoiseCov = (Mat_<double>(2, 2) << 0.4421, 0.3476, 0.3476, 0.2747);
    constantAcceleration.measurementNoiseCov = constantAcceleration.measurementNoiseCov*1; //R*sigma

    //P0
    constantAcceleration.errorCovPost = Mat::eye(6, 6, CV_64F)*100;

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




PersonList::PersonList(int median_window, int numberOfFramesBeforeDestruction, int numberOfFramesBeforeDestructionLocked, double creation_threshold, double validation_gate, double metric_weight, double recognition_threshold, double c_learning_rate, double const_pos_var, double const_vel_var, double const_accel_var)
{
    //This will never get reseted. That will make sure that we have a new id for every new detection
    nPersons = 0;
    this->median_window = median_window;
    this->numberOfFramesBeforeDestruction = numberOfFramesBeforeDestruction;
    this->numberOfFramesBeforeDestructionLocked = numberOfFramesBeforeDestructionLocked;
    this->creation_threshold = creation_threshold;
    this->validation_gate = validation_gate;
    this->metric_weight = metric_weight;
    this->recognition_threshold = recognition_threshold;
    this->c_learning_rate = c_learning_rate;
    this->const_pos_var = const_pos_var;
    this->const_vel_var = const_vel_var;
    this->const_accel_var = const_accel_var;


}

PersonList::~PersonList()
{
    for(std::vector<PersonModel>::iterator it = personList.begin(); it!=personList.end(); it++)
    {
        if(it->mmaeEstimator != NULL)
            delete it->mmaeEstimator;
    }
}

void PersonList::updateDeltaT(double delta_t)
{

    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end();it++)
    {
        it->delta_t = delta_t;
    }

}

void PersonList::predictList()
{
    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end();it++)
    {

        //Predict the right ammount of times to simulate a constant sampling period
        int times = it->delta_t/it->T;
        for(int lol = 0; lol < times; lol++)
            it->mmaeEstimator->predict();

        //Predict the height covariance P_minus. No need to predict the size. We assume
        //its a constant model
        it->heightP += it->heightQ; //
    }
}

void PersonList::updateList()
{

    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end();it++)
    {

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
    }

}

void PersonList::addPerson(Point3d pos, cv::Rect_<int> rect, Mat bvtHistogram)
{
    PersonModel person(pos, rect, nPersons, median_window, bvtHistogram);

    //Initial predict
    int times = person.delta_t/person.T;
    for(int lol = 0; lol < times; lol++)
       person.mmaeEstimator->predict();

    person.metric_weight = this->metric_weight;
    person.const_pos_var = this->const_pos_var;
    person.const_vel_var = this->const_vel_var;
    person.const_accel_var = this->const_accel_var;
    personList.push_back(person);
    nPersons++;

}

void PersonList::associateData(vector<Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects, vector<Mat> colorFeaturesList, vector<Mat> means, vector<Mat> covariances)
{
    //Create distance matrix.
    //Rows represent the detections and columns represent the trackers

    predictList();
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

                //Get x, y State covariance

                Mat aux;
                itcolumn->getCovarianceOfMixture().copyTo(aux);
                Mat errorCovPost = aux(cv::Range(0,2),cv::Range(0,2));

                double height = detectionPos.z*2.0;

                detectionPos.z = 0.0;



                Mat detection2dPos = Mat::zeros(2, 1, CV_32F);
                detection2dPos.at<float>(0,0) = detectionPos.x;
                detection2dPos.at<float>(1,0) = detectionPos.y;

                Mat tracker2dPos = Mat::zeros(2, 1, CV_32F);
                tracker2dPos.at<float>(0,0) = trackerPos.x;
                tracker2dPos.at<float>(1,0) = trackerPos.y;

                Mat detCov = covariances.at(row);

                //Mean of the discretization error is not 0!
                Mat meanDiscreteError = means.at(row);
                meanDiscreteError.convertTo(meanDiscreteError, CV_32F);
                //double dist = norm(trackerPos-detectionPos); // usar distancia de mahalanobis

                //Mahalanobis
                Mat innovationCov = errorCovPost+detCov;
                Mat error = tracker2dPos-detection2dPos;
                double dist = mahalanobisDist(error, meanDiscreteError, innovationCov);


                //Bhattacharya
                //double dist = hellingerDist(detection2dPos, detCov, tracker2dPos, errorCovPost);

                double colorDist;
                colorDist = compareHist(detectionColorHist, trackerColorHist, CV_COMP_BHATTACHARYYA);

               // cout << "Number of detections: " << colorFeaturesList.size() << std::endl;
               // cout << "Color ist of detection " << row << " to tracklet " << (*itcolumn).id << ": " << colorDist << endl;


                if(dist < validation_gate)
                {
                    if(colorDist > recognition_threshold)
                        distMatrixIn[row+col*nDetections] = 1000; //Colors are too different. It cant be the same person...
                    else
                        distMatrixIn[row+col*nDetections] = itcolumn->getScoreForAssociation(height, detectionPos, detCov, detectionColorHist, trackerColorHist);
                }
                else
                {
                    distMatrixIn[row+col*nDetections] = 1000;
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

            //If there is an associated tracker and the distance is less than the validation gate we update the position
            if(assignment[i] != -1)
            {

                Point3d detPos(coordsInBaseFrame.at(i).x, coordsInBaseFrame.at(i).y, 0);
                Point3d trackPos = personList.at(assignment[i]).getPositionEstimate();
                Mat trackerColorHist = personList.at(assignment[i]).getBvtHistogram();
                Mat detectionColorHist = colorFeaturesList.at(i);
                double colorDist = compareHist(detectionColorHist, trackerColorHist, CV_COMP_BHATTACHARYYA);
                trackPos.z = 0.0;

                Mat x = Mat::zeros(2, 1, CV_64F);
                x.at<float>(0,0) = detPos.x;
                x.at<float>(1,0) = detPos.y;



                Mat u = Mat::zeros(2, 1, CV_64F);
                u.at<float>(0,0) = trackPos.x;
                u.at<float>(1,0) = trackPos.y;

                Mat aux;
                personList.at(assignment[i]).getCovarianceOfMixture().copyTo(aux);
                Mat S = aux(cv::Range(0,2),cv::Range(0,2));

                Mat detCov = covariances.at(i);
                Mat meanDiscreteError = means.at(i);
                Mat innovationCov = S+detCov;
                Mat error = x-u;
                double posDist = mahalanobisDist(error, meanDiscreteError, innovationCov);

                if(posDist < validation_gate && colorDist < recognition_threshold)
                {
                    double px = coordsInBaseFrame.at(i).x+meanDiscreteError.at<double>(0,0);
                    double py = coordsInBaseFrame.at(i).y+meanDiscreteError.at<double>(1,0);
                    personList.at(assignment[i]).position.x = px;
                    personList.at(assignment[i]).position.y = py;
                    personList.at(assignment[i]).position.z = coordsInBaseFrame.at(i).z;


                    //Observation covariance update
                    personList.at(assignment[i]).mmaeEstimator->filterBank.at(0).measurementNoiseCov = detCov;
                    personList.at(assignment[i]).mmaeEstimator->filterBank.at(1).measurementNoiseCov = detCov;
                    personList.at(assignment[i]).mmaeEstimator->filterBank.at(2).measurementNoiseCov = detCov;

                    //Bounding boxes...
                    personList.at(assignment[i]).rect = rects.at(i);

                    //Color histograms I'm not going to update it now lol
                    personList.at(assignment[i]).bvtHistogram = personList.at(assignment[i]).bvtHistogram*(c_learning_rate)+(1-c_learning_rate)*colorFeaturesList.at(i);
                }
                else
                {

                    //If there isn't, create a new tracker for each one - IF THERE IS NO OTHER TRACKER IN A ... radius
                    bool existsInRadius = false;

                    Point3d coords = coordsInBaseFrame.at(i);
                    coords.z = 0.0;

                    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
                    {
                        Point3d posEstimate = it->getPositionEstimate();

                        Point3d testPoint(posEstimate.x, posEstimate.y, 0);

                        if(norm(coords-testPoint) < creation_threshold)
                        {
                            existsInRadius = true;
                            break;
                        }
                    }
                    if(!existsInRadius)
                    {
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

                    if(norm(coords-testPoint) < creation_threshold)
                    {
                        existsInRadius = true;
                        break;
                    }
                }
                if(!existsInRadius)
                {
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
        //We create a tracker for each detection, and if there are none, we do nothing! - can't create 2 trackers in the same creation_threshold radius
        for(int i=0; i<nDetections; i++)
        {
            bool existsInRadius = false;

            Point3d coords = coordsInBaseFrame.at(i);
            coords.z = 0.0;

            for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
            {
                Point3d testPoint((*it).positionHistory[0].x, (*it).positionHistory[0].y, 0.0);
                if(norm(coords-testPoint) < creation_threshold)
                {
                    existsInRadius = true;
                    break;
                }
            }
            if(!existsInRadius)
            {
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

cv::Mat PersonList::plotReprojectionAndProbabilities(int targetId, cv::Mat baseFootprintToCameraTransform, Mat K, cv::Mat lastImage)
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

        Mat intrinsics = K;
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
