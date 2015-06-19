/*******************************************************************************
* Pedestrian Detector v0.2    2015-03
* 
* Joao Avelino and Matteo Taiana
* 
* Original code by:
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

/*
 * General includes
 */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

/*
 * Our Includes
 */
#include "sse.hpp"
#include "wrappers.hpp"
#include "opencvInterface.hpp"
#include "chnsPyramid.hpp"
#include "strongClassifierTree.hpp"
#include "readFiles.hpp"

/*
 * OpenCV Includes
 */
#include <opencv2/opencv.hpp>

/*
 * RapidXML include
 */
#include "rapidxml_utils.hpp"

using namespace cv;
using namespace std;

void tic();
void toc();

/*
 * XML meta function
 */
class helperXMLParser {
public:
  // Verbose
  bool verbose;
  
  //////////////////////////////////////////////////////////////////////////////
  // Pyramid calculation options
  //////////////////////////////////////////////////////////////////////////////
  
  // Number of channels of the input image
  int nrChannels;
  // Number of scales to approximate between real scales
  int nrScales;
  // Minimum Dimensions
  int minH; // Height
  int minW; // Width
  
  //////////////////////////////////////////////////////////////////////////////
  // Classifier options
  //////////////////////////////////////////////////////////////////////////////
  
  // WidthOverHeight
  float widthOverHeight;
  // Shrink factor
  int shrinkFactor;
  // Theoretical window dimensions
  int theoWWidth;
  int theoWHeight;
  // Theoretical active window dimensions
  float theoActWWidth;
  float theoActWHeight;
  
  //////////////////  
  // Rectangles Data
  // File with rectangles data
  string rectFile;
  // Number of features
  int nrFeatures;  
  // Number of properties per rectangle
  int nrProp;
    
  //////////////////
  // Classifier Data
  // File with classifier data
  string classFile;  
  // Number of classifiers
  int nrClass;  
  // Number of columns per classifier
  int nrCol;      
  
  // What are these?
  // nBaseFeatures
  int nBaseFeatures;
  // nExtraFeatures
  int nExtraFeatures;

  helperXMLParser(string filename);
  ~helperXMLParser();
  void print();
};


class pedestrianDetector{
    
    public:
    ClassRectangles *rectangles;
    ClassData *classData;
    classifierInput *sctInput;
    int *pad;
    pyrInput *pInput;
    helperXMLParser *parsed;
    
    public: 
      pedestrianDetector(string configuration);
      ~pedestrianDetector();
      vector<cv::Rect_<int> >* runDetector(Mat img_original);
};
