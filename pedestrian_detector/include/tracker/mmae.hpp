#ifndef MMAE_HPP
#define MMAE_HPP

#include <cv.h>
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


class MMAEFilterBank{

public:

    std::vector<KalmanFilter> filterBank;                     //Vector of models
    Mat xMMAE;                                                //Ponderated state
    std::vector<std::vector <int> > modelToxMMAEindexes;      //Since a model might have less states than xMMAE,
                                                              //we will save their corresponding indexes
    bool preComputeA;
    bool commonState;
    bool achievedSteadyState;
    std::vector<Mat> invAk;
    std::vector<double> sqrtDetAk;
    std::vector<double> betas;                                //Beta = 1/((2pi)^(m/2)|Ak|^1/2)
    std::vector<double> probabilities;                        //Probabilities associated with each model




    MMAEFilterBank(std::vector<KalmanFilter> filterBank, std::vector<std::vector <int> > modelToxMMAEindexes, bool commonState = true, bool preComputeA = false, Mat initialState=Mat(), int type = CV_64F);
    ~MMAEFilterBank();
    void predict(Mat control = Mat());
    void correct(Mat &measurement);

private:
    void linkXMMAEtoFilterStates();
    int type;

};

#endif // MMAE_HPP
