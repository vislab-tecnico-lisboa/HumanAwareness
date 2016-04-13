#ifndef __MATCV__H__
#define __MATCV__H__
 
#include "mex.h"
#include "matrix.h"
#include <cstring>
#include "opencv2/opencv.hpp"

#include "boost/bimap.hpp"
 
class Converter{
public:
 Converter(mxArray* src, bool Interleve=true);
 Converter(cv::Mat& src);
 
 operator cv::Mat();
 operator mxArray*();
 
private:
 enum{MATLAB_MXARRAY,OPENCV_MAT} _id;
 bool _interleve;
 
 cv::Mat opencv;
 mxArray* matlab;
 
 
 typedef boost::bimap<mxClassID,unsigned char> bmtype;
 bmtype _idmap;
 void _initidmap();
 
 void _matlab2opencv();
 void _opencv2matlab();
};
 
#endif //__MATCV__H__