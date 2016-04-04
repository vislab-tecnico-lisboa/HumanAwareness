#ifndef __RESOURCE_ALLOCATION__
#define __RESOURCE_ALLOCATION__

#include "knapsack.hpp"
#include <fstream>
#include <iostream>
#include <stack>
#include <ctime>
#include <boost/shared_ptr.hpp>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>




class DARP
{
private:
    void computeValues(const cv::Mat & state_means,
            const cv::Mat & state_variances);
    
    double gaussian(const cv::Mat & x,
            const cv::Mat & mu,
            const cv::Mat & cov);
    
    std::vector<int> optimize(const std::vector<double> & value,
            const std::vector<int> & weight);
public:
    DARP(const int & width_,
            const int & height_,
            const double & max_relative_capacity_=0.2,
            const int & max_items_=7,
            const int & min_width_=52,
            const int & min_height_=128);
    
    std::vector<cv::Rect> getROIS(const cv::Mat & state_means,
            const cv::Mat & state_variances);
    
    std::vector<cv::Mat> getProbabilityMaps()
    {
        return probability_maps;
    }
       
    int width;
    int height;
    double total_capacity;
    double max_relative_capacity;
    double max_items;
    boost::shared_ptr<Knapsack> optimizer;
    
    std::vector<int> item_weight;
    std::vector<double> item_value;
    
    std::vector<cv::Mat> probability_maps;
    std::vector<cv::Rect> rois;
    cv::Mat label_map;
    
    double confidence_scale;
    double min_width;
    double min_height;
    
    // auxs
    cv::Mat x;
    cv::Mat std_dev;
    cv::Mat centroid_std_dev;
    cv::Mat size_std_dev;
    cv::Mat size_means;
    cv::Mat thresholds;
    cv::Mat cov;
protected:
    std::stack<clock_t> tictoc_stack;
    
    void tic() 
    {
        tictoc_stack.push(clock());
    }
    
    double toc_()
    {
        double time_elapsed=((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
        //std::cout << "Time elapsed: " << time_elapsed << std::endl;
        tictoc_stack.pop();
        
        return time_elapsed;
    }
    
};
#endif