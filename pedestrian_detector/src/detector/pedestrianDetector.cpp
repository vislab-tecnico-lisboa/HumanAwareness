/*******************************************************************************
* Pedestrian Detector v0.2    2015-03
* 
*
* Joao Avelino and Matteo Taiana
* 
* Original code by:
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
* 
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/


#include "../include/detector/pedestrianDetector.hpp"
#include <iostream>
#include <ctime>
#include <stack>
#include <ros/package.h>
#include <sstream>

using namespace std;

std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}


/*/ Defining our parser error handler (not defined by library)
void rapidxml::parse_error_handler(const char *what, void *where){
  std::cout << "Parse error: " << what << "\n";
  std::abort();
}
//*/

//*/
helperXMLParser::helperXMLParser(string filename){
  // Read the source file
  ifstream in(filename.c_str());
  
  if(in.fail())
    {
	  std::cout << "There was a problem opening " << filename << endl;
	  exit(-1);
	}

  // Prepare it for RapidXML parser
  std::vector<char> buffer((std::istreambuf_iterator<char>(in)), 
                            std::istreambuf_iterator<char>( ));
  buffer.push_back('\0');
  
  // Rapidxml document class and parser
  rapidxml::xml_document<> doc;
  doc.parse<0>(&buffer[0]);

  // Find our root node
	rapidxml::xml_node<> *root_node = doc.first_node("detector");
	
	// Verbose node
	rapidxml::xml_node<> *verboseN = root_node->first_node("verbose");
	verbose = (atoll(verboseN->first_attribute("value")->value()) != 0);
	
	// Pyramid node
	rapidxml::xml_node<> *pyramidN = root_node->first_node("pyramid");
	nrChannels = atoll(pyramidN->first_attribute("nrChannels")->value());
  nrScales = atoll(pyramidN->first_attribute("nrScales")->value());
  minH = atoll(pyramidN->first_attribute("minH")->value());
  minW = atoll(pyramidN->first_attribute("minW")->value());
  
  // Classifier node
	rapidxml::xml_node<> *classifierN = root_node->first_node("classifier");
	widthOverHeight = atof(classifierN->first_attribute("widthOverHeight")->value());
	shrinkFactor = atoll(classifierN->first_attribute("shrinkFactor")->value());
	
	theoWWidth = atoll(classifierN->first_attribute("theoWWidth")->value());
	theoWHeight = atoll(classifierN->first_attribute("theoWHeight")->value());
	
	theoActWWidth = atof(classifierN->first_attribute("theoActWWidth")->value());
	theoActWHeight = atof(classifierN->first_attribute("theoActWHeight")->value());
	
	rectFile = classifierN->first_attribute("rectFile")->value();
	nrFeatures = atoll(classifierN->first_attribute("nrFeatures")->value());
	nrProp = atoll(classifierN->first_attribute("nrProp")->value());
	
	
	
	classFile = classifierN->first_attribute("classFile")->value();
	
	//With this we don't really need to worry about our working directory
	stringstream ss;
	ss << ros::package::getPath("pedestrian_detector") << "/" << classFile;
	classFile = ss.str();
    /**********************************************************************/
	nrClass = atoll(classifierN->first_attribute("nrClass")->value());
	nrCol = atoll(classifierN->first_attribute("nrCol")->value());
	
	nBaseFeatures = atoll(classifierN->first_attribute("nBaseFeatures")->value());
	nExtraFeatures = atoll(classifierN->first_attribute("nExtraFeatures")->value());
}

helperXMLParser::~helperXMLParser(){

}

void helperXMLParser::print(){
  cout << "Verbose            : " << verbose          << endl
       << "Nr. channels       : " << nrChannels       << endl
       << "Nr. scales         : " << nrScales         << endl
       << "Minimum height     : " << minH             << endl
       << "Minimum width      : " << minW             << endl
       << "widthOverHeight    : " << widthOverHeight  << endl
       << "Shrink factor      : " << shrinkFactor     << endl
       << "Theo. width        : " << theoWWidth       << endl
       << "Theo. height       : " << theoWHeight      << endl
       << "Theo. Act. width   : " << theoActWWidth    << endl
       << "Theo. Act. height  : " << theoActWHeight   << endl
       << "Rect. file         : " << rectFile         << endl
       << "Nr. features       : " << nrFeatures       << endl
       << "Nr. properties     : " << nrProp           << endl
       << "Class. file        : " << classFile        << endl
       << "Nr. class.         : " << nrClass          << endl
       << "Nr. cols           : " << nrCol            << endl
       << "nBaseFeatures      : " << nBaseFeatures    << endl
       << "nExtraFeatures     : " << nExtraFeatures   << endl ;
}


pedestrianDetector::pedestrianDetector(string configuration){

/*Detector initialization*/

  parsed = new helperXMLParser(configuration);
  if(parsed->verbose)
    parsed->print();
    
    pInput = new pyrInput();
    pInput->nApprox = -1; 								//parse it
    pInput->lambdas = new float[3]{0, 0.1105, 0.1083};  //parse it

  /*
   * Prepares the Strong Classifier Inputs
   */
  // Rectangles Data
  rectangles = new ClassRectangles(parsed->rectFile,
                                                    parsed->nrFeatures, 
                                                    parsed->nrProp);
  
  // Classifier Data
  classData = new ClassData(parsed->classFile,
                                       parsed->nrClass, 
                                       parsed->nrCol);
  
  // Setup the classifier
  sctInput = new classifierInput(classData, 
                                                  rectangles,
                                                  parsed->verbose,
                                                  parsed->widthOverHeight,
                                                  parsed->shrinkFactor,
                                                  parsed->theoWWidth,
                                                  parsed->theoWHeight,
                                                  parsed->theoActWWidth,
                                                  parsed->theoActWHeight,
                                                  parsed->nBaseFeatures,
                                                  parsed->nExtraFeatures
                                                 );

//padding
  delete [] (pInput->pad);
  pInput->pad = new int[2]{sctInput->theoreticalVerticalPadding, sctInput->theoreticalHorizontalPadding};
}


pedestrianDetector::~pedestrianDetector(){

    delete(rectangles);
    delete(classData);
    delete(sctInput);
    delete(pInput);
    delete(parsed);

}

vector<cv::Rect_<int> >* pedestrianDetector::runDetector(Mat img_original){
    
  // These are helper variables
  Mat image, imagef, imageO = img_original;
  
  
  
  /*
   * Converts Image to RGB
   * TODO :: This must be remade as image may\may not be in BGR
   */
  cvtColor(imageO, image, CV_BGR2RGB);
  image.convertTo(imagef, CV_32FC3, 1/255.0, 0);

  /*
   * Initialize image properties (misalign - controls memory mis-alignment)
   */
  const int h=imagef.size[0], w=imagef.size[1], misalign=1;
  int c = image.channels();
  
  /*
   * Converts Mat image to float*
   */
  float *img;
  img = convertFromMat(imagef, h, w, c, misalign);

  /*
   * Prepares Pyramid Input
   */
  int *sz = new int[3]{ h, w, c };
  
  // Image Size
  delete [] (pInput->sz);
  pInput->sz = sz;
  
  // Minimum Dimensions
  delete [] (pInput->minDs);
  int *minDs = new int[2]{parsed->minH,parsed->minW};
  pInput->minDs = minDs;
  
  
  /*
   * Calculate Pyramids
   */
  pyrOutput *pOutput = chnsPyramid(img, pInput);
  
  /*
   * Free img memory
   */
   wrFree(img-misalign);
   

    /*
   * Running the detector
   */
  vector<cv::Rect_<int> >* rects = sctRun(pOutput, sctInput);
  
  
  
  delete pOutput;
  return rects;
}
