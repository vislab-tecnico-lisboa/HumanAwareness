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
#include <boost/shared_ptr.hpp>
#include "resource_constraint/resource_allocation.h"


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
        if(nrhs!=7)
        {
            mexErrMsgTxt("wrong inputs number (should be 4)");
        }
        
        double width=*(double *) mxGetPr(prhs[1]);
        double height=*(double *) mxGetPr(prhs[2]);
        double capacity_percentage=*(double *) mxGetPr(prhs[3]);
        double max_items=*(double *) mxGetPr(prhs[4]);
        double min_width=*(double *) mxGetPr(prhs[5]);
        double min_height=*(double *) mxGetPr(prhs[6]);


        DARP * darp_=new  DARP((int)width,(int)height,capacity_percentage,(int)max_items,(int)min_width,(int)min_height);
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
    
    
    if (!strcmp("get_probability_maps", cmd))
    {
        // Check parameters
        //std::cout <<nrhs<<std::endl;
        
        if (nrhs !=2)
            mexErrMsgTxt("detect: Unexpected arguments.");
        
        plhs[0]=MxArray(darp->probability_maps);
        return;
    }
    
    // Call the various class methods
    if (!strcmp("compute_probabilities", cmd))
    {
        // Check parameters
        
        if (nrhs !=5)
            mexErrMsgTxt("detect: Unexpected arguments.");
        const mwSize* size=mxGetDimensions(prhs[2]);
        //std::cout << "size:" << size[1] << " " << size[0] << std::endl;
        cv::Mat state_means=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[2]),0);
        cv::Mat state_variances=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[3]),0);
        cv::Mat left_upper_corners=cv::Mat(size[1],size[0],CV_64F,mxGetData(prhs[4]),0);
        //std::cout << "state_means: " << state_means << std::endl;
        //std::cout << "state_variances: " << state_variances << std::endl;
        //std::cout << "left_upper_corner: " << left_upper_corners << std::endl;
        
        tic();
        std::vector<cv::Rect> rois=darp->getROIS(
                state_means,
                state_variances);
        double time_elapsed=toc_();
        std::cout << "time elapsed (optimization):" << time_elapsed<< std::endl;

        plhs[0]=MxArray(rois);
        plhs[1]=mxCreateDoubleScalar(time_elapsed);

        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
