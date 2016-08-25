#include "../include/tracker/utils.hpp"
#include <ros/ros.h>

double mahalanobisDist(cv::Mat& x, cv::Mat& u, cv::Mat& S)
{

x.convertTo(x, CV_64F);
u.convertTo(u, CV_64F);
S.convertTo(S, CV_64F);

Mat xMinusU = x-u;

Mat invCov;

invert(S, invCov);

Mat results = xMinusU.t()*invCov*xMinusU;

return results.at<double>(0,0);

}


double bhattacharyyaDist(cv::Mat& u1, cv::Mat& S1, cv::Mat& u2, cv::Mat& S2)
{
    u1.convertTo(u1, CV_64F);
    u2.convertTo(u2, CV_64F);
    S1.convertTo(S1, CV_64F);
    S2.convertTo(S2, CV_64F);

    Mat Sigma = (S1+S2)/2;
    Mat error = u1-u2;
    
    Mat invSigma;

    invert(Sigma, invSigma);
    double detSigma = determinant(Sigma);
    double detS1 = determinant(S1);
    double detS2 = determinant(S2);

    Mat result1 = 1.0/8.0*error.t()*invSigma*error;
    
    double result2 = 1.0/2.0*log(detSigma/sqrt(detS1*detS2));
    
    return result1.at<double>(0,0)+result2;

}

double hellingerDist(cv::Mat& u1, cv::Mat& S1, cv::Mat& u2, cv::Mat& S2)
{

    double bat = bhattacharyyaDist(u1, S1, u2, S2);
    
    double bc = exp(-bat);
    
    return sqrt(1-bc);

}

void computeMeasurementStatistics(Mat K, Mat transform, std::vector<double> lambdas, std::vector<cv::Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects,std::vector<cv::Mat> &meansArray, std::vector<cv::Mat> &covMatrices)
{

    Mat RT = transform(Range(0, 3), Range(0, 4));
    Mat P = K*RT;

    //Make direct homography. delete 3rd column
    Mat hDir(3, 3, 6);

    P.col(0).copyTo(hDir.col(0));
    P.col(1).copyTo(hDir.col(1));
    P.col(3).copyTo(hDir.col(2));


    hDir.convertTo(hDir, CV_32FC1);

    Mat H;
    invert(hDir, H);

    vector<cv::Rect_<int> >::iterator itRect = rects.begin();
    vector<double>::iterator itLambdas = lambdas.begin();

    for(vector<Point3d>::iterator itCoords = coordsInBaseFrame.begin(); itCoords != coordsInBaseFrame.end();)
    {

        int Qscx = (int) round(itRect->width*(pow(2.0, 1.0/8.0)-1)/2);
        int Qscy = (int) round(itRect->height*(pow(2.0, 1.0/8.0)-1));

        Qscx = Qscx*2;
        Qscy = Qscy*2;

        double Exapprox = (pow(Qscx,2))/12*((pow(H.at<float>(2,0), 2)*itCoords->x-H.at<float>(0,0)*H.at<float>(2,0))/pow(*itLambdas, 2)) +(pow(Qscy,2))/12*((pow(H.at<float>(2,1), 2)*itCoords->x-H.at<float>(0,1)*H.at<float>(2,1))/pow(*itLambdas, 2));
        double Eyapprox = (pow(Qscx,2))/12*((pow(H.at<float>(2,0), 2)*itCoords->y-H.at<float>(1,0)*H.at<float>(2,0))/pow(*itLambdas, 2))+(pow(Qscy,2))/12*((pow(H.at<float>(2,1), 2)*itCoords->y-H.at<float>(1,1)*H.at<float>(2,1))/pow(*itLambdas, 2));


        //While I don't know the bug... fixed covariances


        Mat mean = (Mat_<double>(2, 1) << Exapprox, Eyapprox);

        Mat cov = (Mat_<double>(2, 2) << 0.4421, 0.3476, 0.3476, 0.2747);

        meansArray.push_back(mean);
        covMatrices.push_back(cov);

        itCoords++;
        itRect++;
        itLambdas++;
    }
}

