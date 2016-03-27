#include "../include/detector/colorFeatures.hpp"
#include <opencv/highgui.h>
#include <opencv/cv.h>

using namespace std;

//inputImage must be in the BGR color space

void extractBVT(cv::Mat& inputImage, cv::Mat& bvtHistogram, int bgBins, std::vector<cv::Rect> partMasks)
{

    cv::Mat hsv;
    std::vector<cv::Mat> matOfHistograms;

    int hBins = 10;
    int sBins = 10;

    //Convert the image to the HSV color space
    cvtColor(inputImage, hsv, CV_BGR2HSV);

/* ---------- Copy/paste from OpenCV website: THIS NEEDS ATTENTION ----------- */

    //Number of levels
    int histSize[] = {hBins, sBins};
    // hue varies from 0 to 179, see cvtColor
    float hRanges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sRanges[] = { 0, 256 };
    const float* ranges[] = { hRanges, sRanges };
    int channels[] = {0, 1};


/*----------------------------------------------------------------------------*/


    //We save each 1D histogram as a row of a matrix for easier "vectorization"
    //Get the histogram for each part
    for(std::vector<cv::Rect>::iterator it = partMasks.begin(); it != partMasks.end(); it++)
    {

        cv::Mat hs_hist;
        //Get our region of interest: this way no data is copied and we have faster processing times!
        cv::Mat imageROI = hsv(*it);
        //Extract V channel
        cv::Mat V(imageROI.rows, imageROI.cols, imageROI.depth());
        //Extract V channel


        //Different bin sizes for each range of Saturation Values - Not used. Worst results
        /*
        *   ________________________________________
        *  | Saturation range | Number of Hue bins |
        *   ---------------------------------------
        *  |      [0 - 8]     |         2          |
        *   ---------------------------------------
        *  |     [8 - 16]     |         4          |
        *   ---------------------------------------
        *  |     [16 - 32]    |         8          |
        *   ---------------------------------------
        *  |     [32 - 64]    |         16         |
        *   ---------------------------------------
        *  |    [64 - 128]    |        32          |
        *   ---------------------------------------
        *  |    [128 - 256]   |        64          |
        *   ---------------------------------------
        */


        // forming an array of matrices is a quite efficient operation,
        // because the matrix data is not copied, only the headers
        cv::Mat out[] = {V};
        int from_to[] = {2, 0};

        cv::mixChannels(&imageROI, 1,out, 1, from_to, 1);

        //Mask out black spots
        cv::Mat maskBlackSpots;
        cv::threshold(V, maskBlackSpots, 1, 1, CV_THRESH_BINARY);


        //H-S histogram

       hs_hist.empty();
       cv::calcHist(&imageROI, 1, channels, maskBlackSpots, hs_hist, 2, histSize, ranges, true, false);

       hs_hist = hs_hist.reshape(0, 1);


        //Black value - count the number of black pixels
        float totalNumberOfPixels = imageROI.rows*imageROI.cols;

        float black_value = totalNumberOfPixels-cv::countNonZero(V);

        //Gray histogram
        cv::Mat gray_histogram;
        int gray_channels[] = {0};
        int gray_histSize[] = {bgBins};
        float grayRanges[] = {0, 256};
        const float* gRanges[] = {grayRanges};

        cv::calcHist(&V, 1, gray_channels, maskBlackSpots, gray_histogram, 1, gray_histSize, gRanges, true, false);
        gray_histogram = gray_histogram.reshape(0, 1);

        //Normalize histograms
        float normalizationConstant = (hsv.rows*hsv.cols)/(imageROI.rows*imageROI.cols);

        black_value = black_value*normalizationConstant;

        //Merge
        matOfHistograms.push_back(hs_hist*normalizationConstant);
        matOfHistograms.push_back(cv::Mat(1,1, CV_32FC1, black_value)*2.32);
        matOfHistograms.push_back(gray_histogram*normalizationConstant*0.36);
    }



    //Merge each single histogram associated to each part in a single full histogram
    cv::hconcat(matOfHistograms, bvtHistogram);


}
