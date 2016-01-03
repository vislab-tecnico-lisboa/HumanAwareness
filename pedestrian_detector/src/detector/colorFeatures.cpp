#include "../include/detector/colorFeatures.hpp"
#include <highgui.h>
#include <cv.h>
#include <ros/ros.h>

using namespace std;

//inputImage must be in the BGR color space

void extractBVT(cv::Mat& inputImage, cv::Mat& bvtHistogram, int bgBins, std::vector<cv::Rect> partMasks)
{

    cv::Mat hsv;
    std::vector<cv::Mat> matOfHistograms;

    //Convert the image to the HSV color space
    cvtColor(inputImage, hsv, CV_BGR2HSV);

/* ---------- Copy/paste from OpenCV website: THIS NEEDS ATTENTION ----------- */

    // hue varies from 0 to 179, see cvtColor
    float hRanges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
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


        //Different bin sizes for each range of Saturation Values
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

        // [0 - 8] range
        float sRanges_1[] = { 0, 8 };
        int histSize_1[] = {2, 1};
        const float* ranges_1[] = { hRanges, sRanges_1};
        cv::Mat hs_hist_1;
        cv::calcHist(&imageROI, 1, channels, maskBlackSpots, hs_hist_1, 2, histSize_1, ranges_1, true, false);

        hs_hist_1 = hs_hist_1.reshape(0, 1);

        // [8 -16] range
        float sRanges_2[] = { 8, 16 };
        int histSize_2[] = {4, 1};
        const float* ranges_2[] = { hRanges, sRanges_2 };
        cv::Mat hs_hist_2;

        cv::calcHist(&imageROI, 1, channels, maskBlackSpots, hs_hist_2, 2, histSize_2, ranges_2, true, false);

        hs_hist_2 = hs_hist_2.reshape(0, 1);

        // [16 - 32] range
        float sRanges_3[] = { 16, 32 };
        int histSize_3[] = {8, 1};
        const float* ranges_3[] = { hRanges, sRanges_3 };
        cv::Mat hs_hist_3;

        cv::calcHist(&imageROI, 1, channels, maskBlackSpots, hs_hist_3, 2, histSize_3, ranges_3, true, false);

        hs_hist_3 = hs_hist_3.reshape(0, 1);

        // [32 - 64] range
        float sRanges_4[] = { 32, 64 };
        int histSize_4[] = {16, 1};
        const float* ranges_4[] = { hRanges, sRanges_4 };
        cv::Mat hs_hist_4;

        cv::calcHist(&imageROI, 1, channels, maskBlackSpots, hs_hist_4, 2, histSize_4, ranges_4, true, false);

        hs_hist_4 = hs_hist_4.reshape(0, 1);

        // [64 - 128] range
        float sRanges_5[] = { 64, 128 };
        int histSize_5[] = {32, 1};
        const float* ranges_5[] = { hRanges, sRanges_5 };
        cv::Mat hs_hist_5;

        cv::calcHist(&imageROI, 1, channels, maskBlackSpots, hs_hist_5, 2, histSize_5, ranges_5, true, false);

        hs_hist_5 = hs_hist_5.reshape(0, 1);

        // [128 - 256] range
        float sRanges_6[] = { 128, 256 };
        int histSize_6[] = {64, 1};
        const float* ranges_6[] = { hRanges, sRanges_6 };
        cv::Mat hs_hist_6;

        cv::calcHist(&imageROI, 1, channels, maskBlackSpots, hs_hist_6, 2, histSize_6, ranges_6, true, false);

        hs_hist_6 = hs_hist_6.reshape(0, 1);

        std::vector<cv::Mat> hist_vector;

        hist_vector.push_back(hs_hist_1);
        hist_vector.push_back(hs_hist_2);
        hist_vector.push_back(hs_hist_3);
        hist_vector.push_back(hs_hist_4);
        hist_vector.push_back(hs_hist_5);
        hist_vector.push_back(hs_hist_6);

        cv::hconcat(hist_vector, hs_hist);


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
