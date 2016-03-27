#include "mc_convert.hpp"
 
Converter::Converter(mxArray* src,bool Interleve):matlab(src),_id(MATLAB_MXARRAY),_interleve(Interleve){
    _initidmap();
}
Converter::Converter(cv::Mat& src):opencv(src),_id(OPENCV_MAT){
    _initidmap();
}
 
Converter::operator cv::Mat()
{
    if(_id==OPENCV_MAT)
        return(opencv);
    _matlab2opencv();
    return(opencv);
}
 
Converter::operator mxArray*()
{
    if(_id==MATLAB_MXARRAY)
        return(matlab);
    _opencv2matlab();
    return(matlab);
}
 
void Converter::_initidmap()
{
    _idmap.insert(bmtype::value_type(mxINT8_CLASS,CV_8S));
    _idmap.insert(bmtype::value_type(mxUINT8_CLASS,CV_8U));
    _idmap.insert(bmtype::value_type(mxINT16_CLASS,CV_16S));
    _idmap.insert(bmtype::value_type(mxUINT16_CLASS,CV_16U));
    _idmap.insert(bmtype::value_type(mxINT32_CLASS,CV_32S));
    _idmap.insert(bmtype::value_type(mxSINGLE_CLASS,CV_32F));
    _idmap.insert(bmtype::value_type(mxDOUBLE_CLASS,CV_64F));
}
 
void  Converter::_opencv2matlab()
{
    //Is the data type supported?
    bmtype::right_map::const_iterator itr=_idmap.right.find(opencv.depth());
     
    //if not then
    if(itr==_idmap.right.end())
    {
        mexErrMsgTxt("OpenCV2Matlab:Unsupported data type.");
    }
     
    //Find the matlab data type
    mxClassID type=itr->second;
     
    //We support max 3 dimensions
    mwSignedIndex dims[3];
    dims[0]=opencv.rows;
    dims[1]=opencv.cols;
    dims[2]=opencv.channels();
     
    //if number of channels is 1, its a 2D array
    if(dims[0]>0 && dims[1]>0 && dims[2]==1)
    {
        //Create the array to be returned
        matlab=mxCreateNumericArray(2,dims,type,mxREAL);
        //Create opencv header for the matlab data
        cv::Mat tmp=cv::Mat(dims[1],dims[0],CV_MAKETYPE(opencv.depth(),1),mxGetData(matlab),0);
        //Transpose the opencv data to get row major data for matlab
        tmp=opencv.t();
         
        const mwSize* size=mxGetDimensions(matlab);
        mxAssert((opencv.rows==size[0])&(opencv.cols==size[1]),"OpenCV2Matlab:Conversion mismatch");
    }
    else
    {
        //Create the array to be returned
        matlab=mxCreateNumericArray(3,dims,type,mxREAL);
         
        //Seperate the channels
        std::vector<cv::Mat> chans(dims[2]);
        //cv::split(opencv,&chans[0]);
        cv::split(opencv,chans);
         
        //Create opencv header as a "flat" image for the matlab data
        cv::Mat tmp=cv::Mat(dims[1]*dims[2],dims[0],CV_MAKETYPE(opencv.depth(),1),mxGetData(matlab),0);
         
        for(int i=0;i<dims[2];i++)
        {
            //transpose the opencv channels image to row major matlab data
            cv::Mat tmp2=chans[i].t();
            //Copy the data to the flat matlab data
            tmp2.copyTo(tmp.rowRange(i*dims[1],(i+1)*dims[1]));
        }
         
        const mwSize* size=mxGetDimensions(matlab);
        mxAssert((opencv.rows==size[0])&(opencv.cols==size[1])&(opencv.channels()==size[2]),"OpenCV2Matlab:Conversion mismatch");
    }  
}
 
void  Converter::_matlab2opencv()
{
    //find the corresponding corresponding opencv data type
    bmtype::left_map::const_iterator itr=_idmap.left.find(mxGetClassID(matlab));
     
    //No corresponding type?
    if(itr==_idmap.left.end()| mxIsComplex(matlab) | mxIsSparse(matlab))
    {
        std::string msg="Matlab2OpenCV:Unsupported data type:"+(itr==_idmap.left.end()?std::string(mxGetClassName(matlab)):"")
        +(mxIsComplex(matlab)?" Complex":"")+(mxIsSparse(matlab)?" Sparse":".");
        mexErrMsgTxt(msg.c_str());
    }
     
    unsigned char type=itr->second;
     
    //Get number of dimensions
    const mwSize dims=mxGetNumberOfDimensions(matlab);
     
    //Cannot handle more that 3 dimensions
    if(dims>3)
    {
        std::ostringstream o;
        o<<"Matlab2OpenCV:Supports upto 3 dimensions. You supplied "<<dims<<".";
        mexErrMsgTxt(o.str().c_str());
    }
     
    //Get actual dimensions
    const mwSize* size=mxGetDimensions(matlab);
     
    //Check if 2 or 3 dimentions
    if(dims==2)
    {
        //Create header for row major matlab data
        const cv::Mat donotmodify=cv::Mat(size[1],size[0],CV_MAKETYPE(type,1),mxGetData(matlab),0);
         
        //Transpose the data so that it is column major for opencv.
        opencv=donotmodify.t();
        mxAssert((opencv.rows==size[0])&(opencv.cols==size[1]),"Matlab2OpenCV:Conversion mismatch");
    }
    else
    {
        //Create header for the "flat" matlab data
        const cv::Mat donotmodify=cv::Mat(size[1]*size[2],size[0],CV_MAKETYPE(type,1),mxGetData(matlab),0);
         
        //Transpose the flat data
        cv::Mat flat=donotmodify.t();
         
        //Create vector of channels to pass to opencv merge operataion
        std::vector<cv::Mat> chans(size[2]);
         
        for(int i=0;i<size[2];i++)
        {
            chans[i]=flat.colRange(i*size[1],(i+1)*size[1]);
        }
        cv::merge(chans,opencv);
        mxAssert((opencv.rows==size[0])&(opencv.cols==size[1])&(opencv.channels()==size[2]),"Matlab2OpenCV:Conversion mismatch");
    }
     
    
}