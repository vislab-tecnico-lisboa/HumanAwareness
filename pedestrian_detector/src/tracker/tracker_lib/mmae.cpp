#include "../include/tracker/mmae.hpp"

MMAEFilterBank::~MMAEFilterBank()
{
}
MMAEFilterBank::MMAEFilterBank(std::vector<KalmanFilter> & filterBank,
                               std::vector<std::vector <int> > & modelToxMMAEindexes,
                               bool commonState_,
                               bool preComputeA_,
                               Mat initialState,
                               int type_) :
    filterBank(filterBank),
    modelToxMMAEindexes(modelToxMMAEindexes),
    invAk(filterBank.size(), Mat()),
    sqrtDetAk(filterBank.size(), 0),
    betas(filterBank.size(), 0),
    probabilities(filterBank.size(), ((double) 1)/ filterBank.size()),
    preComputeA(preComputeA_),
    commonState(commonState_),
    type(type_)
{
    int xMMAEsize = 0;

    CV_Assert(filterBank.size() == modelToxMMAEindexes.size()); //Check if we have a map for each filter
    CV_Assert(initialState.type() == CV_32F || initialState.type() == CV_64F);

    //Check the size of each Kalman Filter. The size of the ponderated state will be the size of the biggest state
    int counter=0;
    for(std::vector<KalmanFilter>::iterator it=filterBank.begin(); it != filterBank.end(); it++, counter++)
    {
        CV_Assert(it->statePost.rows == (int) modelToxMMAEindexes.at(counter).size());

        if(it->statePost.rows > xMMAEsize)
            xMMAEsize = it->statePost.rows;
    }

    if(initialState.empty())
        xMMAE = Mat::zeros(xMMAEsize, 1, type);
    else
    {
        CV_Assert(initialState.rows == xMMAEsize && initialState.cols == 1);
        xMMAE = initialState;
    }

    stateCovMMAE = Mat::zeros(xMMAEsize, xMMAEsize, type);

    counter = 0;

    for(std::vector<KalmanFilter>::iterator it=filterBank.begin(); it != filterBank.end(); it++, counter++)
    {
        Mat tmpMat = Mat::zeros(xMMAEsize, xMMAEsize, type);
        it->errorCovPost.copyTo(tmpMat(cv::Range(0, it->errorCovPost.rows), cv::Range(0, it->errorCovPost.cols)));

        Mat tmpState = Mat::zeros(xMMAEsize, 1, type);
        it->statePost.copyTo(tmpState(cv::Range(0, it->statePost.rows), cv::Range(0, it->statePost.cols)));


        stateCovMMAE = stateCovMMAE + probabilities.at(counter)*(tmpMat+tmpState*tmpState.t()-xMMAE*xMMAE.t());
    }




    //Now initialize all filters with the initial state
    linkXMMAEtoFilterStates();
}

void MMAEFilterBank::linkXMMAEtoFilterStates()
{

    CV_Assert(modelToxMMAEindexes.size() == filterBank.size());
    std::vector<std::vector<int> >::iterator itIndexes = modelToxMMAEindexes.begin();
    for(std::vector<KalmanFilter>::iterator itFilter = filterBank.begin(); itFilter!= filterBank.end() && itIndexes != modelToxMMAEindexes.end(); itFilter++, itIndexes++)
    {
        CV_Assert( (int) itIndexes->size() == itFilter->statePost.rows);
        int i = 0;
        for(std::vector<int>::iterator itPos = itIndexes->begin(); itPos != itIndexes->end(); itPos++, i++)
        {
            if(type == CV_32F)
                itFilter->statePost.at<float>(i, 0) = xMMAE.at<float>(*itPos, 0);
            else if(type == CV_64F)
                itFilter->statePost.at<double>(i, 0) = xMMAE.at<double>(*itPos, 0);
        }
    }
}


/*Makes the prediction step on all filters, and gathers the auxiliary values*/

void MMAEFilterBank::predict(Mat control)
{
    if(preComputeA)
    {

    }
    else
    {
        int i = 0;

        int fbSize = filterBank.size();
        int invAkSize = invAk.size();
        int sqrtDetAkSize = sqrtDetAk.size();

        CV_Assert(fbSize == invAkSize && invAkSize == sqrtDetAkSize);

        for(std::vector<KalmanFilter>::iterator itFilter = filterBank.begin(); itFilter != filterBank.end(); itFilter++, i++)
        {
            itFilter->predict(control);

            itFilter->errorCovPre = itFilter->errorCovPre+Mat::eye(itFilter->errorCovPre.rows, itFilter->errorCovPre.cols, itFilter->errorCovPre.type())*0.003;

            //Aux vars
            Mat H = itFilter->measurementMatrix;
            Mat Pminus = itFilter->errorCovPre;
            Mat R = itFilter->measurementNoiseCov;
            Mat Htransp;
            transpose(H, Htransp);
            Mat Ak = H*Pminus*Htransp+R;
            invert(Ak, invAk.at(i));
            double akdet = determinant(Ak);
            sqrtDetAk.at(i) = sqrt(akdet);
            betas.at(i) = 1.0/(2*CV_PI*sqrtDetAk.at(i));


        }
    }

}

/*Performs the correction step if there is a measurement, computes mmae estimation*/

void MMAEFilterBank::correct(Mat &measurement)
{

    std::vector<double> densities;
    double density;

    if(!measurement.empty()) //If there is a measurement, perform correction step for all filters
    {                        //If no measure is available we assume the previous probabilities for the models
        CV_Assert(filterBank.size() == probabilities.size() && probabilities.size() == betas.size() && betas.size() == invAk.size());
        int i = 0;
        double sumOfAll = 0;
        for(std::vector<KalmanFilter>::iterator itFilter = filterBank.begin(); itFilter != filterBank.end(); itFilter++, i++)
        {
            if(measurement.type() != type)
            {
                measurement.convertTo(measurement, type);
            }


            Mat residue = measurement-itFilter->measurementMatrix*itFilter->statePre;
            residue.convertTo(residue, CV_64F);
            Mat trResidue;
            transpose(residue, trResidue);
            Mat q = trResidue*invAk.at(i)*residue;
            q.convertTo(q, CV_64F);
            Mat expTerm = ((-1.0/2.0))*q;

            cv::exp(expTerm, expTerm);

            itFilter->correct(measurement);

            //VERIFICAR SE O EXPTERM DA UM ESCALAR OU SE HOUVE ALGUM ERRO!


            CV_Assert(expTerm.rows == 1 && expTerm.cols == 1);

            if(expTerm.type() == CV_32F)
            {
                density = betas.at(i)*static_cast<double>(expTerm.at<float>(0, 0));
            }
            else if(expTerm.type() == CV_64F)
            {
                density = betas.at(i)*expTerm.at<double>(0, 0);
            }


            densities.push_back(density);
            sumOfAll+=density*probabilities.at(i);

        }


        //We only update the probabilities if we dont have an outlier
        if(sumOfAll > pow(10, -170))
        {
            CV_Assert(probabilities.size() == densities.size());
            double sumProbs = 0;
            int j = 0;
            for(std::vector<double>::iterator itProb = probabilities.begin(); itProb != probabilities.end();
                itProb++, j++)
            {

                (*itProb) = densities.at(j)*(*itProb)/sumOfAll;

                *itProb+=0.01;
                sumProbs += *itProb;
            }

            for(std::vector<double>::iterator itProb = probabilities.begin(); itProb != probabilities.end();
                itProb++, j++)
            {
                (*itProb) = (*itProb)/sumProbs;
            }


        }
    }

    //Get the ponderated state vector
    Mat xMMAE_tmp = Mat::zeros(xMMAE.rows, xMMAE.cols, type);

    std::vector<std::vector<int> >::iterator itIndexes = modelToxMMAEindexes.begin();
    int filtNumber = 0;
    for(std::vector<KalmanFilter>::iterator itFilter=filterBank.begin(); itFilter != filterBank.end(); itFilter++, itIndexes++, filtNumber++)
    {
        CV_Assert((int) itIndexes->size() == itFilter->statePost.rows);
        int i = 0;
        for(std::vector<int>::iterator itPos = itIndexes->begin(); itPos != itIndexes->end(); itPos++, i++)
        {
            if(type == CV_32F)
            {
                xMMAE_tmp.at<double>(*itPos, 0) += probabilities.at(filtNumber)*itFilter->statePost.at<double>(i, 0);
            }

            else if(type == CV_64F)
            {
                xMMAE_tmp.at<double>(*itPos, 0) += probabilities.at(filtNumber)*itFilter->statePost.at<double>(i, 0);
            }
        }
    }

    xMMAE = xMMAE_tmp;

    stateCovMMAE = Mat::zeros(stateCovMMAE.rows, stateCovMMAE.rows, type);

    int counter = 0;

    for(std::vector<KalmanFilter>::iterator it=filterBank.begin(); it != filterBank.end(); it++, counter++)
    {
        Mat tmpMat = Mat::zeros(stateCovMMAE.rows, stateCovMMAE.cols, type);
        it->errorCovPost.copyTo(tmpMat(cv::Range(0, it->errorCovPost.rows), cv::Range(0, it->errorCovPost.cols)));

        Mat tmpState = Mat::zeros(xMMAE.rows, xMMAE.cols, type);
        it->statePost.copyTo(tmpState(cv::Range(0, it->statePost.rows), cv::Range(0, it->statePost.cols)));


        stateCovMMAE = stateCovMMAE + probabilities.at(counter)*(tmpMat+tmpState*tmpState.t()-xMMAE*xMMAE.t());
    }


    //
    //We we want to use the same state vector for all filters, and that state vector is the ponderated vector
    //just replace statePost with the ponderated vector in each filter

    if(commonState)
    {
        linkXMMAEtoFilterStates();
    }

}
