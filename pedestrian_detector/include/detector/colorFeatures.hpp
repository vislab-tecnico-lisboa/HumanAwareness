#ifndef COLORFEATURES2_HPP
#define COLORFEATURES2_HPP
#include <cv.h>

void extractBVT(cv::Mat& inputImage, cv::Mat& bvtHistogram, int bgBins, std::vector<cv::Rect> partMasks);


#endif // COLORFEATURES2_HPP
