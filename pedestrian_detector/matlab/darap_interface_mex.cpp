// Mex stuff
#include "mex.h"
#include "mc_convert/mc_convert.hpp"
#include "mex_handle.hpp"
#include "mexopencv.hpp"
#include "MxArray.hpp"
//General purpose includes
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <stack>
#include <string>
#include <sstream>
//#include <thread>
#include <fstream>
#include <iostream>
#include <stack>
#include <ctime>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "detector/pedestrianDetector.hpp"


std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

double toc_()
{
    double time_elapsed=((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    //std::cout << "Time elapsed: " << time_elapsed << std::endl;
    tictoc_stack.pop();
    
    return time_elapsed;
}

class DARP
{
public:
    DARP(const int & width_,
            const int & height_,
            const double & max_capacity_=0.2,
            const int & max_items_=7) :
                width(width_),
                        height(height_),
                        max_capacity(max_capacity_), //percentage
                        max_items(max_items_),
                        total_capacity((double)width_*height_)
                {
                    // ALLOCATE MEMORY FOR AUXILIARY PROBABILITY MAPS
                    item_relative_weight.resize(max_items);
                    item_value.resize(max_items);
                    
                    probability_maps.resize(max_items);
                    
                    // INITIALIZE WITH A GAUSSIAN CENTERED
                    probability_map=cv::Mat(height_,width_,CV_64F,cv::Scalar(0));
                    
                    cv::Mat mu(2,1,CV_64F);
                    mu.at<double>(0,0)=width/2.0;
                    mu.at<double>(1,0)=height/2.0;
                    
                    cv::Mat cov(2,2,CV_64F,cv::Scalar(0.0));
                    cov.at<double>(0,0)=1000.0;
                    cov.at<double>(1,1)=1000.0;
                    
                    // don't go further than this
                    int width_threshold=3*sqrt(cov.at<double>(0,0));
                    int height_threshold=3*sqrt(cov.at<double>(1,1));
                    //std::cout << width_threshold << std::endl;
                    //std::cout << height_threshold << std::endl;
                    

                    tic();
                    
                    int width_begin=-width_threshold+round(mu.at<double>(0,0));
                    if(width_begin<0) width_begin=0;
                    int width_end=width_threshold+round(mu.at<double>(0,0));
                    if(width_end>width) width_end=width;
                    
                    int height_begin=-height_threshold+round(mu.at<double>(1,0));
                    if(height_begin<0) height_begin=0;
                    int height_end=height_threshold+round(mu.at<double>(1,0));
                    if(height_end>height) height_end=height;
                    
                    cv::Mat x(2,1,CV_64F);
                    for(int i=width_begin;i<width_end;++i)
                    {
                        for(int j=height_begin; j<height_end;++j)
                        {
                            x.at<double>(0,0)=i;
                            x.at<double>(1,0)=j;
                            probability_map.at<double>(j,i)=gaussian(x,mu,cov);
                        }
                    }
                    std::cout << toc_()<< std::endl;
                };
                
                void computeProbablities(
                        const cv::Mat & centroid_means,
                        const cv::Mat & centroid_variances,
                        const cv::Mat & size_means,
                        const cv::Mat & size_variances)
                {
                    tic();
                    cv::Mat x(2,1,CV_64F);
                    

                    cv::Mat centroid_std_dev;
                    cv::sqrt(centroid_variances,centroid_std_dev);

                    cv::Mat size_std_dev;
                    cv::sqrt(size_variances,size_std_dev);
                    
                    cv::Mat std_dev;
                    cv::sqrt(centroid_variances+size_variances,std_dev);

                    cv::Mat thresholds=size_means+3.0*std_dev;
                    
                    // For each tracked object...
                    for(int o=0; o<centroid_means.rows;++o)
                    {
                        //probability_maps[o]=cv::Mat(height,width,CV_64F,cv::Scalar(0));

                        if(thresholds.at<double>(o,0)>250||thresholds.at<double>(o,1)>250)
                            continue;
                        // don't go further than this
                        int width_threshold=thresholds.at<double>(o,0);
                        int height_threshold=thresholds.at<double>(o,1);

                        cv::Mat cov(2,2,CV_64F,cv::Scalar(0.0));
                        cov.at<double>(0,0)=std_dev.at<double>(o,0)*std_dev.at<double>(o,0);
                        cov.at<double>(1,1)=std_dev.at<double>(o,1)*std_dev.at<double>(o,1);

                        int width_begin=-width_threshold+round(centroid_means.at<double>(o,0));
                        if(width_begin<0) width_begin=0;
                        int width_end=width_threshold+round(centroid_means.at<double>(o,0));
                        if(width_end>width) width_end=width;

                        int height_begin=-height_threshold+round(centroid_means.at<double>(o,1));
                        if(height_begin<0) height_begin=0;
                        int height_end=height_threshold+round(centroid_means.at<double>(o,1));
                        if(height_end>height) height_end=height;
                        
                        
                        // Compute item relative weight
                        double item_total_weight=4.0*(width_threshold)*(height_threshold);
                        item_relative_weight[o]=item_total_weight/total_capacity;
                        std::cout << " item rel. weight:" << item_relative_weight[o]<< std::endl;
                        
                        // Compute item value
                        item_value[o]=0.0;
                        
                        // Fill simply with the best probability (at the centroid)
                        double best=gaussian(centroid_means.row(o).t(),centroid_means.row(o).t(),cov);
                        
                        for(int u=width_begin;u<width_end;++u)
                        {
                            for(int j=height_begin; j<height_end;++j)
                            {
                                x.at<double>(0,0)=u;
                                x.at<double>(1,0)=j;
                                
                                //probability_maps[o].at<double>(j,u)=best;
                                item_value[o]+=best;
                            }
                        }
                        
                        std::cout << " item value:" << item_value[o]<< std::endl;
                        std::cout << " item normalized rel. value:" << item_value[o]/item_relative_weight[o]<< std::endl;

                        //item_value[o]/=total_capacity; // normalize ??
                    }
                    std::cout << "time elapsed (probability map fill):" << toc_()<< std::endl;
                }
                
                double gaussian(
                        const cv::Mat & x,
                        const cv::Mat & mu,
                        const cv::Mat & cov)
                {
                    cv::Mat diff=x-mu;
                    double det=determinant(cov);
                    double aux=(0.5/M_PI)*(1.0/sqrt(det));
                    
                    cv::Mat exponent=-0.5*diff.t()*cov.inv()*diff;
                    return aux*std::exp(exponent.at<double>(0));
                }
                
                int width;
                int height;
                double total_capacity;
                double max_capacity;
                double max_items;
                
                std::vector<double> item_relative_weight;
                std::vector<double> item_value;
                cv::Mat probability_map;
                std::vector<cv::Mat> probability_maps;
                cv::Mat label_map;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Get the command string
    char cmd[64];
    if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
        mexErrMsgTxt("First input should be a command string less than 64 characters long.");
    
    // New
    if (!strcmp("new", cmd))
    {
        // Check parameters
        //std::cout << "nrhs: " << nrhs << std::endl;
        if(nrhs!=5)
        {
            mexErrMsgTxt("wrong inputs number (should be 4)");
        }
        
        double width=*(double *) mxGetPr(prhs[1]);
        double height=*(double *) mxGetPr(prhs[2]);
        double capacity_percentage=*(double *) mxGetPr(prhs[3]);
        double max_items=*(double *) mxGetPr(prhs[4]);
        DARP * darp_=new  DARP((int)width,(int)height,capacity_percentage,(int)max_items);
        plhs[0] = convertPtr2Mat<DARP>(darp_);
        
        return;
    }
    
// Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
        mexErrMsgTxt("Second input should be a class instance handle.");
    
// Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<DARP>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    DARP *darp = convertMat2Ptr<DARP>(prhs[1]);
    
    
    if (!strcmp("get_probability_map", cmd))
    {
        // Check parameters
        //std::cout <<nrhs<<std::endl;
        
        if (nrhs !=2)
            mexErrMsgTxt("detect: Unexpected arguments.");
        
        plhs[0]=MxArray(darp->probability_map);
        return;
    }
    
    // Call the various class methods
    if (!strcmp("compute_probabilities", cmd))
    {
        // Check parameters
        
        if (nrhs !=6)
            mexErrMsgTxt("detect: Unexpected arguments.");
        const mwSize* size=mxGetDimensions(prhs[2]);
        //std::cout << "size:" << size[1] << " " << size[0] << std::endl;
        cv::Mat centroid_means=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[2]),0);
        cv::Mat centroid_variances=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[3]),0);
        cv::Mat size_means=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[4]),0);
        cv::Mat size_variances=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[5]),0);
        
        //std::cout << "centroid_means: " << centroid_means << std::endl;
        //std::cout << "centroid_variances: " << centroid_variances << std::endl;
        //std::cout << "size_means: " << size_means << std::endl;
        //std::cout << "size_variances: " << size_variances << std::endl;
        
        darp->computeProbablities(
                centroid_means,
                centroid_variances,
                size_means,
                size_variances);
        
        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
