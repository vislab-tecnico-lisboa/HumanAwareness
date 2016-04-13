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

const char *centroid[] = {"x", "y"};
const char *fieldsPoint[] = {"x", "y", "width","height"};


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
            mexErrMsgTxt("wrong inputs number (should be 5)");
        }
        
        std::string conf_pedestrians=mxArrayToString(prhs[1]);
        std::string conf_heads=mxArrayToString(prhs[2]);
        std::string detector_type=mxArrayToString(prhs[3]);
        std::string class_path=mxArrayToString(prhs[4]);
        
        //std::cout << "conf_pedestrians:" << conf_pedestrians << std::endl;
        //std::cout << "conf_heads:" << conf_heads << std::endl;
        //std::cout << "detector_type:" << detector_type << std::endl;
        //std::cout << "class_path:" << class_path << std::endl;
        
        plhs[0] = convertPtr2Mat<pedestrianDetector>(new  pedestrianDetector(conf_pedestrians, conf_heads, detector_type, class_path));
        
        return;
    }
    
// Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
        mexErrMsgTxt("Second input should be a class instance handle.");
    
// Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<pedestrianDetector>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    
    // Get the class instance pointer from the second input
    pedestrianDetector *pedestrian_detector = convertMat2Ptr<pedestrianDetector>(prhs[1]);
    
    // Call the various class methods
    // to_cortical
    if (!strcmp("detect", cmd))
    {
        // Check parameters
        if (nrhs !=7)
            mexErrMsgTxt("detect: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const cv::Mat opencv_const=MxArray(prhs[2]).toMat();
        
        int x=*(double *) mxGetPr(prhs[3]);
        int y=*(double *) mxGetPr(prhs[4]);
        int width=*(double *) mxGetPr(prhs[5]);
        int height=*(double *) mxGetPr(prhs[6]);
        
        // Setup a rectangle to define your region of interest
        cv::Rect myROI(x , y, width, height);
        
        // Crop the full image to that image contained by the rectangle myROI
        // Note that this doesn't copy the data
        cv::Mat croppedRef(opencv_const, myROI);
        
        // Copy the data into new matrix
        //croppedRef.copyTo(image);
        
        
        // Call the method
        tic();
        pedestrian_detector->runDetector(croppedRef);
        double time_elapsed=toc_();
        std::cout << "time elapsed (detection):" << time_elapsed<< std::endl;

        
        mxArray *p;
        int i=0;
        plhs[0] = mxCreateStructMatrix(1, pedestrian_detector->boundingBoxes->size(), 2, centroid);
        plhs[1] = mxCreateStructMatrix(1, pedestrian_detector->boundingBoxes->size(), 4, fieldsPoint);
        
        for (vector<cv::Rect_<int> >::iterator it = pedestrian_detector->boundingBoxes->begin(); it != pedestrian_detector->boundingBoxes->end(); ++it)
        {
            it->x+=x;
            it->y+=y;
            
            // Centroids
            mxSetField(plhs[0], i, "x", mxCreateDoubleScalar(it->x+it->width/2.0));
            mxSetField(plhs[0], i, "y", mxCreateDoubleScalar(it->y+it->height/2.0));

            // ROIs
            mxSetField(plhs[1], i, "x", mxCreateDoubleScalar(it->x));
            mxSetField(plhs[1], i, "y", mxCreateDoubleScalar(it->y));
            mxSetField(plhs[1], i, "width", mxCreateDoubleScalar(it->width));
            mxSetField(plhs[1], i, "height", mxCreateDoubleScalar(it->height));
            ++i;
        }
        
        // Convert from opencv to matlab
        plhs[2]=mxCreateDoubleScalar(time_elapsed);

        return;
    }
    
    if (!strcmp("detect_with_rois", cmd))
    {
        // Check parameters
        if (nrhs !=8)
            mexErrMsgTxt("detect: Unexpected arguments.");
        
        // Convert from matlab to opencv
        const cv::Mat opencv_const=MxArray(prhs[2]).toMat();
        int x=*(double *) mxGetPr(prhs[3]);
        int y=*(double *) mxGetPr(prhs[4]);
        int width=*(double *) mxGetPr(prhs[5]);
        int height=*(double *) mxGetPr(prhs[6]);
        const cv::Mat rois_mat=MxArray(prhs[7]).toMat();
        tic();
        std::vector<cv::Rect> detections;
        for(int i=0; i<rois_mat.rows;++i)
        {
            if(rois_mat.at<double>(i,2)<64||rois_mat.at<double>(i,3)<128)
                continue;
            // Setup a rectangle to define your region of interest
            cv::Rect roi(rois_mat.at<double>(i,0), 
                    rois_mat.at<double>(i,1), 
                    rois_mat.at<double>(i,2), 
                    rois_mat.at<double>(i,3));
        
            // Crop the full image to that image contained by the rectangle myROI
            // Note that this doesn't copy the data
            cv::Mat croppedRef(opencv_const, roi);
            pedestrian_detector->runDetector(croppedRef);
            for(std::vector<cv::Rect>::iterator it=pedestrian_detector->boundingBoxes->begin(); 
            it!=pedestrian_detector->boundingBoxes->end(); ++it)
            {
                (*it).x+=rois_mat.at<double>(i,0);
                (*it).y+=rois_mat.at<double>(i,1);
                detections.push_back(*it);
            }
        }

        double time_elapsed=toc_();
        std::cout << "time elapsed (detection):" << time_elapsed<< std::endl;

        
        // Call the method
        

        
        mxArray *p;
        int i=0;
        plhs[0] = mxCreateStructMatrix(1, detections.size(), 2, centroid);
        plhs[1] = mxCreateStructMatrix(1, detections.size(), 4, fieldsPoint);
        
        for (std::vector<cv::Rect>::iterator it = detections.begin(); it != detections.end(); ++it)
        {
            it->x+=x;
            it->y+=y;
            
            // Centroids
            mxSetField(plhs[0], i, "x", mxCreateDoubleScalar(it->x+it->width/2.0));
            mxSetField(plhs[0], i, "y", mxCreateDoubleScalar(it->y+it->height/2.0));

            // ROIs
            mxSetField(plhs[1], i, "x", mxCreateDoubleScalar(it->x));
            mxSetField(plhs[1], i, "y", mxCreateDoubleScalar(it->y));
            mxSetField(plhs[1], i, "width", mxCreateDoubleScalar(it->width));
            mxSetField(plhs[1], i, "height", mxCreateDoubleScalar(it->height));
            ++i;
        }
        
        // Convert from opencv to matlab
        plhs[2]=mxCreateDoubleScalar(time_elapsed);
        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
