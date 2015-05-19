/*******************************************************************************
* Pedestrian Detector v0.2    2015-04
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Edited by: Joao Avelino
* 
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

#include "../include/chnsCompute.hpp"
#include <ctime>
#include <sys/time.h>
#include <stack>

using namespace std;

std::stack<clock_t> tictoc_stack3;

double time_diff(struct timeval x , struct timeval y)
{
    double x_ms , y_ms , diff;
     
    x_ms = (double)x.tv_sec*1000000 + (double)x.tv_usec;
    y_ms = (double)y.tv_sec*1000000 + (double)y.tv_usec;
     
    diff = (double)y_ms - (double)x_ms;
     
    return diff;
}

void tic3() {
    tictoc_stack3.push(clock());
}

void toc3() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack3.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack3.pop();
}

void convTriAux(float *M, float *&S, int misalign, int height, int width,
		int channels, float r, int s){
    //int downSample = 1; // WARNING : This is the default by dollar

    S = (float*) wrCalloc(height*width*channels+misalign,
	sizeof(float)) + misalign;

    if (r > 0 && r <= 1 && s <= 2){
	float rnew = 12 / r / (r + 2) - 2;
	convTri1(M, S, height, width, channels, rnew, s);
    }else
	convTri(M, S, height, width, channels, round(r), s);
}

pChns::pChns(){
    pColor = new Color();
    pColor->enabled = true;
    pColor->colorSpace = luv;

    pGradMag = new GradMag();
    pGradMag->enabled = true;
    pGradMag->colorChn = 0;
    pGradMag->normRad = 5;
    pGradMag->normConst = 0.005f;

    pGradHist = new GradHist();
    pGradHist->enabled = true;
    pGradHist->binSize = 1;
    pGradHist->nOrients = 6;
    pGradHist->softBin = false;
    pGradHist->useHog = false;
    pGradHist->clipHog = 0.2f;

    pCustom = new Custom();
    pCustom->enabled = true;
    pCustom->name = "REQ";
    pCustom->hFunc = "REQ";
    pCustom->pFunc = NULL;
    pCustom->padWith = 0;

    complete = false;
}

pChns::~pChns(){

delete pCustom;
delete pGradHist;
delete pGradMag;
delete pColor;
}

infoOut::infoOut(pChns *_input, int _width,	int _height, int _chn,
		int _chnTrans, int _widthH, int _heightH, float *_I, float *_M,
		float *_H, int misalign){
    int nOrients = _input->pGradHist->nOrients;
    bool useHog = _input->pGradHist->useHog;

    width = _width;
    height = _height;

    widthH = _widthH;
    heightH = _heightH;


    input = _input;

    enableColor = _input->pColor->enabled;
    enableGradMag = _input->pGradMag->enabled;
    enableGradHist = _input->pGradHist->enabled;

    nTypes =  enableColor + enableGradMag + enableGradHist;

    data = (imgWrap**) wrCalloc(nTypes, sizeof(imgWrap*));

    I = new imgWrap(_I, _width, _height, _chn, misalign);
    M = new imgWrap(_M, _width, _height, _chnTrans, misalign);

    if (!useHog)
	H = new imgWrap(_H, widthH, heightH, _chnTrans * nOrients, misalign);
    else
	H = new imgWrap(_H, widthH, heightH, _chnTrans * nOrients * 4, misalign);


    if(enableColor)
	data[0] = I;

    if(enableGradMag)
	data[1] = M;

    if(enableGradHist)
	data[2] = H;
}


infoOut* chnsCompute(float* image, int height, int width, int channels,
		pChns *pchns){

/*
 * Variable Declaration
 */
    const int sf=sizeof(float), misalign=1;
    float *I, *M, *H, *O, *G; //TODO: allocate memory
    int chnTrans = 1; //Number of channels on each transformation
    infoOut *output;

/*
 * Get default parameters pChns
 */
    //TODO :: Need to parse inputs

/*
 * Extract parameters from pChns
 */
    //TODO :: Need to parse inputs


/*
 * Compute color channels
 */
    I = rgbConvertMeta(image, height, width, channels, misalign,
	1
	);

/*
 * Compute gradient magnitude channel
 */
    if( pchns->pGradMag->enabled ){

	M  = (float*) wrCalloc(height*width*chnTrans+misalign, sf) + misalign;

	O = 0;
	if ( pchns->pGradHist->enabled )
	    O  = (float*) wrCalloc(height*width*chnTrans+misalign,
		sf) + misalign;

	int lastChannel = 0;
	int colorChn = pchns->pGradMag->colorChn;
	if (colorChn > 0 && colorChn < channels)
	    lastChannel = height*width*(channels-1);

	gradMag( image + lastChannel, M, O, height, width, channels);

	if ( pchns->pGradMag->normRad > 0 ){
	    int downSample = 1; // WARNING : This is the default by dollar
	    int r = pchns->pGradMag->normRad, s = downSample;

	    float *S = 0;
	    convTriAux(M, S, misalign, height, width, chnTrans, r, s);

	    gradMagNorm(M, S, height, width, pchns->pGradMag->normConst);
	    wrFree(S-misalign);
	}
    }

/*
 * Compute gradient histogram channels
 */
    int hb = 0;
    int wb = 0;
    if ( pchns->pGradHist->enabled ){
	int binSize = pchns->pGradHist->binSize;
	int nOrients = pchns->pGradHist->nOrients;
	bool softBin = pchns->pGradHist->softBin;
	bool useHog = pchns->pGradHist->useHog;

	hb = height / binSize;
	wb = width / binSize;

	H  = (float*) wrCalloc(hb*wb*chnTrans*nOrients + misalign,
	    sf) + misalign;
	
	gradHist(M, O, H, height, width, binSize, nOrients, softBin);

	if (useHog){
	    G  = (float*) wrCalloc(hb*wb*chnTrans*nOrients*4 + misalign,
		sf) + misalign;

	    float clipHog = pchns->pGradHist->clipHog;
	    hog( H, G, height, width, binSize, nOrients, clipHog);
	    wrFree(H-misalign);

	    H = G;
	}
    }

/*
 * Compute custom channels
 */
    //TODO : There was no need to implement any.

/*
 * Construct extra info for output struct
 */
    //TODO : Is this necessary?

/*
 * Create output struct
 */
    output = new infoOut(
	    pchns,
	    width,
	    height,
	    channels,
	    chnTrans,
	    wb,
	    hb,
	    I,
	    M,
	    H,
	    misalign
	    );

    /*
     * Clean up unecessary arrays
     */
    if(O) wrFree(O-misalign);

    return output;
}
