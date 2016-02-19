/*******************************************************************************
* Pedestrian Detector v0.2    2015-04
*
* Fabio Reis, Matteo Taiana and Joao Avelino
* [freis-at-isr.ist.utl.pt] and [mtaiana-at-isr.ist.utl.pt]
* 
* Original code by Matteo Taiana targeted for Matlab.
*
* Modified for Aggregate Channel Features
* 
* Please email us if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/detector/strongClassifierTree.hpp"
#include <sys/time.h>
#include <ros/ros.h>
#include <stack>

double *classifierData  = NULL; //global variable, accessible from everywhere
int nWeakClassifiers = 0;



/*
 * Classifier Input constructor
 */
classifierInput::classifierInput(ClassData *classifier,
                                 ClassRectangles *rect,
                                 bool _verbose,
                                 float _widthOverHeight,
                                 int _shrinkFactor,
                                 int _theoreticalWindowWidth,
                                 int _theoreticalWindowHeight,
                                 float _theoreticalActiveWindowWidth,
                                 float _theoreticalActiveWindowHeight,
                                 int _nBaseFeatures,
                                 int _nExtraFeatures
                                 ){
    classData = classifier;
    classRect = rect;

    widthOverHeight = _widthOverHeight;
    shrinkFactor = _shrinkFactor;

    theoreticalWindowWidth = _theoreticalWindowWidth;
    theoreticalWindowHeight = _theoreticalWindowHeight;
    theoreticalActiveWindowWidth  = _theoreticalActiveWindowWidth;
    theoreticalActiveWindowHeight = _theoreticalActiveWindowHeight;

    windowWidth = floor( theoreticalWindowWidth  / shrinkFactor );
    windowHeight = floor( theoreticalWindowHeight / shrinkFactor );

    activeWindowWidth  =  floor( theoreticalActiveWindowWidth  / shrinkFactor );
    activeWindowHeight =  floor( theoreticalActiveWindowHeight / shrinkFactor );

    windowHorizontalPadding = ceil((windowWidth - activeWindowWidth) / 2);
    windowVerticalPadding   = ceil((windowHeight - activeWindowHeight) / 2);

    theoreticalHorizontalPadding = ceil((theoreticalWindowWidth - theoreticalActiveWindowWidth) / 2);
    theoreticalVerticalPadding   = ceil((theoreticalWindowHeight - theoreticalActiveWindowHeight) / 2);


    imageHorizontalPadding	= windowHorizontalPadding + 1;
    imageVerticalPadding	= windowVerticalPadding + 1;
    verticalSuperPadding	= 0;
    horizontalSuperPadding	= 0;

    returnFeatures	= true;
    verbose			= _verbose;
    performNMS		= true;

    // 1. Get data for classifierData
    nClassifiers = classData->nRows;
    nWeakClassifiers = classData->nCols;
    nFeatures = classRect->nRows;
    nRectRows = classRect->nRows;
    nRectCols = classRect->nCols;
    
    // This values?
    nBaseFeatures = _nBaseFeatures;
    
    
    //ACF Lookup Table initialization
    
    for(int featureId = 0; featureId < nFeatures; featureId++){
        featureLUT[featureId][0] = (int)featureId/(windowWidth*windowHeight); // c
        featureLUT[featureId][1] = (int)(featureId-featureLUT[featureId][0]*windowWidth*windowHeight)/windowHeight; // x
        featureLUT[featureId][2] = (int)featureId-windowHeight*featureLUT[featureId][1]-featureLUT[featureId][0]*windowHeight*windowWidth; // y
	}
}


/*
 * Detection Structure for output
 */
struct Detection
{
    double U0;
    double V0;
    double U1;
    double V1;
    double confidence;
};

/*
 * Function to compare Detections
 */
bool compareDetections(Detection d1, Detection d2){
    return (d1.confidence>d2.confidence);
}

// Functions to access classifier data, please take care that these pretain 
// directly to the rectangles.dat file used! Where there are 14 cols.
double alpha(int row){
    int col=13;
    return classifierData[row*nWeakClassifiers+ col];
}

int feature(int row){
    int col=1;
    return classifierData[row*nWeakClassifiers+ col];
}

double threshold(int row){
    int col=2;
    return classifierData[row*nWeakClassifiers+ col];
}

int direction(int row){
    int col=3;
    return classifierData[row*nWeakClassifiers+ col];
}

int featureSat(int row){
    int col=5;
    return classifierData[row*nWeakClassifiers+ col];
}

double thresholdSat(int row){
    int col=6;
    return classifierData[row*nWeakClassifiers+ col];
}

int directionSat(int row){
    int col=7;
    return classifierData[row*nWeakClassifiers+ col];
}

int featureNotSat(int row){
    int col=9;
    return classifierData[row*nWeakClassifiers+ col];
}

double thresholdNotSat(int row){
    int col=10;
    return classifierData[row*nWeakClassifiers+ col];
}

int directionNotSat(int row){
    int col=11;
    return classifierData[row*nWeakClassifiers+ col];
}


/*
 * Strong Classifier Tree (sct)
 */
vector<cv::Rect_<int> >* sctRun(pyrOutput *outputPyr, classifierInput *cInput)
{
	int (*featureLUT)[3]=cInput->featureLUT;
    /*
     * Input explicit variables declaration
     */
    imgWrap ***pyrData = outputPyr->chnsPerScale;

    bool verbose = cInput->verbose;

    classifierData = cInput->classData->classifiers;
    
    nWeakClassifiers = cInput->nWeakClassifiers;
    int nFeatures = cInput->nFeatures;
    int nClassifiers = cInput->nClassifiers;
    int nBaseFeatures = cInput->nBaseFeatures;
    
    int windowWidth = cInput->windowWidth;
    int windowHeight = cInput->windowHeight;
    float theoreticalActiveWindowWidth = cInput->theoreticalActiveWindowWidth;
    float theoreticalActiveWindowHeight = cInput->theoreticalActiveWindowHeight;
    int horizontalSuperPadding = cInput->horizontalSuperPadding;
    int verticalSuperPadding = cInput->verticalSuperPadding;

    int nScales = outputPyr->nScales;
    int nChannels = outputPyr->nChannels;
    float *scales = outputPyr->scales;


    /*
     * Variables declaration
     */
    int row = 0;
    int col = 0;
    int scaleId = -1;
    int classifierId = -1;
    int nDetections = 0;
    float *data;
    int nRows;
    int nCols;
    imgWrap *currentScaleData;


    //DEBUG
    if(verbose){
	cout<<"**********************************************************"<<endl;
	cout<<"Initialization:"<<endl;
	cout<<"nFeatures      = "<<nFeatures   <<endl;
	cout<<"nScales        = "<<nScales     <<endl;
	cout<<"windowWidth    = "<<windowWidth <<endl;
	cout<<"windowHeight   = "<<windowHeight<<endl;
	cout<<"theoreticalActiveWindowWidth  = "<<theoreticalActiveWindowWidth<<endl;
	cout<<"theoreticalActiveWindowHeight = "<<theoreticalActiveWindowHeight<<endl;
	cout<<"windowHorizontalPadding  = "<<cInput->windowHorizontalPadding<<endl;
	cout<<"windowVerticalPadding = "<<cInput->windowVerticalPadding<<endl;
	cout<<"nClassifiers   = "<< nClassifiers<<endl;
	cout<<"**********************************************************"<<endl;
    }

    //Detections to ouput
    list<Detection> detections;
    Detection currentDetection;

    /*
     * Begin cycling through all the scales and running the detector on them.
     */
    
    for(scaleId=0; scaleId<nScales; scaleId++){
	//For all scales
	//Get size of the images for this size and the pointer to the data

	//All the channels are concatenated we get the first and only imgWrap.
	currentScaleData = pyrData[scaleId][0];
	data = currentScaleData->image; //Get the pointer the data
	nRows = currentScaleData->height;
	nCols = currentScaleData->width;
	

	//Only detect when the image is bigger than the detection window
    if((nRows < windowHeight) || (nCols < windowWidth))
	    continue; //skip this size: it's too small to use our detector

	//*************************************
	// 1. Run the soft cascade on the data
	//*************************************
	
	
	double weakClass = 0;
	for (col = 0; col<(nCols - windowWidth); col++ ){
	    for (row = 0; row<(nRows - windowHeight); row++ ){
		//Run the detector on this window
		double confidence = 0;
		for (classifierId = 0; classifierId<nClassifiers; classifierId++){

		    // Compute the value of the 3 features associated with this
		    // weak classifier
		    // 1. Root
		    int featureId = classifierData[classifierId*nWeakClassifiers+ 1];
		    double thresholdValue = classifierData[classifierId*nWeakClassifiers+ 2];
		    double directionValue = classifierData[classifierId*nWeakClassifiers+ 3];
		    double featureValue;
/*		      int x, y, c;
		      
			  
              c = cInput->featureLUT[featureId][0];
              x = cInput->featureLUT[featureId][1];
              y = cInput->featureLUT[featureId][2];*/

              
              featureValue = data[featureLUT[featureId][0]*nRows*nCols+(row+featureLUT[featureId][2])+(col+featureLUT[featureId][1])*nRows]; //acf -> pixel lookup at a scale
			  

		        if( (featureValue - thresholdValue) * directionValue >=0 ){
			    // 2. Satisfy leaf

			    int featureIdSat         = classifierData[classifierId*nWeakClassifiers+ 5];
			    double thresholdValueSat = classifierData[classifierId*nWeakClassifiers+ 6];
			    double directionValueSat = classifierData[classifierId*nWeakClassifiers+ 7];
			    double featureValueSat;

 /*               int x, y, c;
				c = cInput->featureLUT[featureIdSat][0];
				x = cInput->featureLUT[featureIdSat][1];
				y = cInput->featureLUT[featureIdSat][2];*/
              
                featureValueSat = data[featureLUT[featureIdSat][0]*nRows*nCols+(row+featureLUT[featureIdSat][2])+(col+featureLUT[featureIdSat][1])*nRows]; //acf -> pixel lookup at a scale 
				

			    if( (featureValueSat - thresholdValueSat) * directionValueSat >=0 )
			        weakClass =  1;
			    else
			        weakClass = -1;
		    }else{
			// 3. Not Satisfy leaf

			int featureIdNotSat         = classifierData[classifierId*nWeakClassifiers+ 9];
			double thresholdValueNotSat = classifierData[classifierId*nWeakClassifiers+ 10];
			double directionValueNotSat = classifierData[classifierId*nWeakClassifiers+ 11];
			double featureValueNotSat;
 /*
			int x, y, c;
			c = featureLUT[featureIdNotSat][0];
			x = featureLUT[featureIdNotSat][1];
			y = featureLUT[featureIdNotSat][2];
            */
            featureValueNotSat =  data[featureLUT[featureIdNotSat][0]*nRows*nCols+(row+featureLUT[featureIdNotSat][2])+(col+featureLUT[featureIdNotSat][1])*nRows];

			if( (featureValueNotSat - thresholdValueNotSat) * directionValueNotSat >=0 )
			    weakClass =  1;
			else
			    weakClass = -1;
		    }

		    confidence = confidence + weakClass*classifierData[classifierId*nWeakClassifiers+ 13];

		    //WARNING Euristic value from Dollar
		    if(confidence < magicThreshold)
			break;
		} //For each classifier
		
		if(confidence>0){
		    nDetections++;

		    //Pre - BMVC
		    //*/ col and row are the coordinates in the shrinked, padded image
		    double V0 = ( (double)(row) *4  - verticalSuperPadding) / scales[scaleId] ; //both window and image are padded: this takes care of itself, I don't have to bother
		    double U0 = ( (double)(col) *4  - horizontalSuperPadding) / scales[scaleId] ; //both window and image are padded: this takes care of itself, I don't have to bother
		    double V1 = V0 + ( theoreticalActiveWindowHeight/ scales[scaleId] );
		    double U1 = U0 + ( theoreticalActiveWindowWidth  / scales[scaleId] );
		    //detections(nDetections,:) = [V0,U0,V1,U1,confidence];
		    //*/
            
		    if(verbose)
			printf("Detection #%d. Scale = %d, scaling = %f, "
			    "Col = %d, Row = %d, U0 = %f, V0 = %f\n",
			    nDetections, scaleId, scales[scaleId],
			    col, row, U0, V0
			    );

		    //Add the bounding box + confidence to the results that will be returned
		    currentDetection.V0         = V0;
		    currentDetection.V1         = V1;
		    currentDetection.U0         = U0;
		    currentDetection.U1         = U1;
		    currentDetection.confidence = confidence;
		    detections.push_back(currentDetection);

		} //else do nothing, discard the window
	    } //scan image rows
	} //scan image columns
	
    }
	
	
    //*************************************
    // 2. Non-Maximal Suppression Part
    //*************************************
	
    //NMS code ported from some Matlab code by Piotr Dollár, see his Matlab toolbox for indications on which papers to cite, if you decide to use this code.
    bool performNms = true;
    bool greedy = true;
    bool ovrDnm = false;
    double overlap = 0.65;

    if(nDetections>0){
	if(performNms){
	    //Sort the "detections" structure according to the confidence value, in descending order
	    detections.sort(compareDetections);

	    //Initialize the keep vector
	    vector<bool> keep(nDetections, true); //initialize all keep elements to true.

	    //Compute area of each BB
	    vector<double> areas(nDetections);
	    list<Detection>::iterator itDetections;
	    vector<double>::iterator itV;
	    itV=areas.begin();
	    for(itDetections=detections.begin(); itDetections!=detections.end(); ++itDetections){
		*itV = ( (*itDetections).U1 - (*itDetections).U0 ) * ( (*itDetections).V1 - (*itDetections).V0 );
		itV++;
	    }

	    vector<Detection>detVector(detections.begin(), detections.end()); //copy the content of the list of detections to a vector

	    for(int i=0; i<nDetections; i++){
		if(greedy && keep[i]==false) continue; //If this bb has already been discarded, let's ignore it and skip to the next

		for(int j=i+1; j<nDetections; j++){
		    if(keep[j]==false) continue; //No need to compare with the previously discarded examples
		    double iw = min(detVector[i].U1, detVector[j].U1 ) - max(detVector[i].U0, detVector[j].U0);
		    if(iw<=0) continue; //No horizontal overlap -> no need to compare these two bb's
		    double ih = min(detVector[i].V1, detVector[j].V1 ) - max(detVector[i].V0, detVector[j].V0); ;
		    if(ih<=0) continue; //No vertical overlap -> no need to compare these two bb's
		    double o = iw*ih; //Compute overlap area
		    double u;
		    if(ovrDnm)
			u = areas[i]+areas[j]-o; //Denominator of the fraction  = union of bb's "i" and "j", PASCAL rule
		    else
			u = min(areas[i],areas[j]); //Denominator of the fraction = smallest area between "i" and "j"

		    o = o/u; //Compute the ratio of the areas
		    if(o>overlap) //When two bb's match, suppress the least confident one
			keep[j]=false;
		}
	    }

	    //Remove the elements with keep() == false from the list
	    vector<bool>::iterator itKeep;
	    itDetections=detections.begin();
	    for(itKeep = keep.begin(); itKeep!=keep.end(); ++itKeep){
		//cout<<(*itDetections).confidence<<" ";
		if((*itKeep)==false){
		    //cout<<"Erasing!\n";
		    itDetections = detections.erase(itDetections);
		}else{
		    //cout<<"Keeping.\n";
		    itDetections++;
		}
	    }

	    nDetections = detections.size();
	    if(verbose)
  	    cout<<"nDetections="<<nDetections<<endl;
	}
    }
    
    vector<cv::Rect_<int> > *listRect = new vector<cv::Rect_<int> >();
    list<Detection>::iterator itDetections;
    for(itDetections=detections.begin(); itDetections!=detections.end(); ++itDetections){
        if(verbose)
          cout << itDetections->confidence 
               << ", (" << itDetections->U0 
               << ", " << itDetections->V0 
               << ") x (" 
               <<  itDetections->U1 
               << ", " 
               << itDetections->V1 
               << ")" << endl;
               
        int ax = itDetections->U0;
        int ay = itDetections->V0;
        int aw = itDetections->U1 - ax;
        int ah = itDetections->V1 - ay;
        listRect->push_back(Rect_<int>(ax, ay, aw, ah));
    }
    return listRect;
}
