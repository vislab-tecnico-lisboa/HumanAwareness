#ifndef MMAE_HPP
#define MMAE_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

/*Thank you stackoverflow for this tip!*/

template<int I>
struct openCVType {};

template<>
struct openCVType<CV_64F> { typedef double type_t; };
template<>
struct openCVType<CV_32F> { typedef float type_t; };


class MMAEFilterBank
{

public:
    Mat xMMAE;                                                //Ponderated state
    Mat stateCovMMAE;                                         //Covariance of the mixture of Gaussians
    std::vector<KalmanFilter> filterBank;                     //Vector of models
    std::vector<std::vector <int> > modelToxMMAEindexes;      //Since a model might have less states than xMMAE,
    //we will save their corresponding indexes

    std::vector<Mat> invAk;
    std::vector<double> sqrtDetAk;
    std::vector<double> betas;                                //Beta = 1/((2pi)^(m/2)|Ak|^1/2)
    std::vector<double> probabilities;                        //Probabilities associated with each model
    bool preComputeA;
    bool commonState;
    bool achievedSteadyState;

    MMAEFilterBank(std::vector<KalmanFilter> & filterBank, std::vector<std::vector <int> > &  modelToxMMAEindexes, bool commonState_ = true, bool preComputeA_ = false, Mat initialState=Mat(), int type_ = CV_64F);
    ~MMAEFilterBank();
    void predict(Mat control = Mat());
    void correct(Mat &measurement);

private:
    void linkXMMAEtoFilterStates();
    int type;

};

#endif // MMAE_HPP
