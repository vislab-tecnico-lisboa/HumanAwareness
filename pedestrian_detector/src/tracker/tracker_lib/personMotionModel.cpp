#include "../include/tracker/personMotionModel.hpp"
#include <algorithm>
#include <vector>
#include "../include/HungarianFunctions.hpp"

/*
* Some motion models are implemented here in order to try to
* get better position estimates, based on the velocity and
* previous position of the person.
*
* TODO: Kalman filtering and motion prediction?
*/

Mat PersonModel::getBvtHistogram()
{
  return bvtHistogram;
}


void PersonModel::updateVelocityArray(Point3d detectedPosition)
{
    for(int i=0; i < 25; i++)
        velocity[25-i] = velocity[25-(i+1)];

    if(!deadReckoning)
    {
      if(positionHistory[1].x != -1000 && positionHistory[1].y != -1000)
      {
      velocity[0].x = -(detectedPosition.x - positionHistory[1].x)/delta_t;
      velocity[0].y = -(detectedPosition.y - positionHistory[1].y)/delta_t;
      }
      else
      {
          Point2d vel;
          vel = velocityMedianFilter();
          velocity[0].x = vel.x;
          velocity[0].y = vel.y;
      }

    }
    else
    {
        //Dead reckoning.
        velocity[0].x = velocity[1].x;
        velocity[0].y = velocity[1].y;
    }

    //ROS_ERROR_STREAM("Id:" << id << " | velx: " << velocity[0].x << " | vely: " << velocity[0].y);

    //This must be better thinked...

}


Point3d PersonModel::getPositionEstimate()
{
    //return filteredPosition+filteredVelocity*delta_t;

    return medianFilter();
}


void PersonModel::updateModel()
{

 //Maybe applying a median filter to the velocity gives good predictive results

  //updateVelocityArray(detectedPosition); - not used yet
  ros::Time now = ros::Time::now();
  ros::Duration sampleTime = now - lastUpdate;

  delta_t = sampleTime.toSec();
  lastUpdate = now;

  for(int i=0; i < median_window-1; i++)
      positionHistory[median_window-1-i] = positionHistory[median_window-1-(i+1)];

  positionHistory[0] = position;

  updateVelocityArray(position);

  position.x = -1000;
  position.y = -1000;
  position.z = 0.95;


  for(int i=0; i < 4; i++)
      rectHistory[4-i] = rectHistory[4-(i+1)];

  rectHistory[0] = rect;

  //velocity in m/ns

  filteredVelocity = velocityMedianFilter();


}

PersonModel::PersonModel(Point3d detectedPosition, cv::Rect_<int> bb, int id, int median_window, Mat bvtHistogram)
{

  this->median_window = median_window;
  toBeDeleted = false;

//  positionHistory = new Point2d[median_window];


    for(int i=0; i < 25; i++)
        velocity[i] = Point2d(0, 0);

  position = detectedPosition;

  this->bvtHistogram = bvtHistogram;


  rect = bb;
  rectHistory[0] = bb;

  positionHistory[0] = detectedPosition;


  for(int i=1; i < median_window; i++)
  {
      positionHistory[i].x =-1000;
      positionHistory[i].y = -1000;
      positionHistory[i].z = 0.95;
  }


  lockedOnce = false;

  this->id = id;
  noDetection = 0;

  delta_t = 0.1;
  lastUpdate = ros::Time::now();

  deadReckoning = false;

}

Point3d PersonModel::getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point3d estimation)
{
    Point3d nearest(-1000, -1000, 0);
    Point3d estimation3d(estimation.x, estimation.y, 0);

    if(coordsInBaseFrame.size() == 0)
        return nearest;

    double distance=1000000;

    for(vector<cv::Point3d>::iterator it = coordsInBaseFrame.begin(); it != coordsInBaseFrame.end(); it++)
      {
        //It's dumb and slow to create matrix headers just to wrap 2 points...
        //I'm leaving this commented just to remember that!
        // double dist = norm(est3d, Mat((*it)), NORM_L2);

        double dist = norm(estimation3d-(*it));
        if(dist < distance && dist < 0.5)
        {
            distance = dist;
            nearest = (*it);
        }

      }
    return nearest;
}

Point3d PersonModel::medianFilter()
{
    //Optimize this later

    double x[median_window];
    double y[median_window];
    double z[median_window];

    for(int i = 0; i<median_window; i++)
    {
        x[i] = positionHistory[i].x;
        y[i] = positionHistory[i].y;
        z[i] = positionHistory[i].z;
    }

    vector<double> x_vect(x, x + sizeof(x)/sizeof(x[0]));
    vector<double> y_vect(y, y + sizeof(y)/sizeof(y[0]));
    vector<double> z_vect(z, z + sizeof(z)/sizeof(y[0]));

    std::sort(x_vect.begin(), x_vect.begin() + median_window);
    std::sort(y_vect.begin(), y_vect.begin() + median_window);
    std::sort(z_vect.begin(), z_vect.begin() + median_window);

    Point3d medianPoint(x_vect.at((int) (median_window/2)), y_vect.at((int) (median_window/2)), z_vect.at((int) (median_window/2)));

    return medianPoint;

}

Point2d PersonModel::velocityMedianFilter()
{
    //Optimize this later

    double x[25];
    double y[25];

    for(int i = 0; i<25; i++)
    {
        x[i] = velocity[i].x;
        y[i] = velocity[i].y;
    }

    vector<double> x_vect(x, x + sizeof(x)/sizeof(x[0]));
    vector<double> y_vect(y, y + sizeof(y)/sizeof(y[0]));

    std::sort(x_vect.begin(), x_vect.begin() + 5);
    std::sort(y_vect.begin(), y_vect.begin() + 5);


    Point2d medianPoint(x_vect.at(2), y_vect.at(2));

    return medianPoint;

}

PersonModel::~PersonModel()
{
//    delete [] positionHistory;
}

PersonList::PersonList(int median_window, int numberOfFramesBeforeDestruction, int numberOfFramesBeforeDestructionLocked, double associatingDistance)
{
  //This will never get reseted. That will make sure that we have a new id for every new detection
  this->numberOfFramesBeforeDestruction = numberOfFramesBeforeDestruction;
    this->numberOfFramesBeforeDestructionLocked = numberOfFramesBeforeDestructionLocked;
    this-> associatingDistance = associatingDistance;
  nPersons = 0;
  this->median_window = median_window;

}

void PersonList::updateList()
{
  //TODO

    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end();)
    {

        if((*it).position.x != -1000 && (*it).position.y != -1000)
        {
          (*it).noDetection = 0;
          (*it).deadReckoning = false;
        }
        else
        {
          (*it).noDetection++;
          //Dead reckoning - We can't see it but we will assume it will maitain the same velocity
            (*it).deadReckoning = true;
          (*it).position = (*it).getPositionEstimate();

        }
        (*it).updateModel();

        if(((*it).noDetection > numberOfFramesBeforeDestruction && (*it).lockedOnce==false) || ((*it).noDetection > numberOfFramesBeforeDestructionLocked && (*it).lockedOnce==true))
        {
            it->toBeDeleted = true;
        }

            it++;
    }

}

void PersonList::addPerson(Point3d pos, cv::Rect_<int> rect, Mat bvtHistogram)
{

   PersonModel person(pos, rect, nPersons, median_window, bvtHistogram);
   personList.push_back(person);
   nPersons++;

}

void PersonList::associateData(vector<Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects, vector<Mat> colorFeaturesList)
{
    //Create distance matrix.
    //Rows represent the detections and columns represent the trackers


    int nTrackers = personList.size();
    int nDetections = coordsInBaseFrame.size();

    if(nTrackers > 0 && nDetections > 0)
    {

    double *distMatrixIn = (double*) malloc(sizeof(double)*nTrackers*nDetections);
    double *assignment = (double*) malloc(sizeof(double)*nDetections);
    double *cost = (double*) malloc(sizeof(double)*nDetections);

    //Fill the matrix
    /*  _           _
    *  |  l1c1 l1c2  |
    *  |  l2c1 l2c2  |
    *  |_ l3c1 l3c2 _|
    *
    *  [l1c1, l2c1, l3c1, l1c2,l2c2,l3c2]
    */

    int row = 0;

    //Each detection -> a row
    for(vector<Point3d>::iterator itrow = coordsInBaseFrame.begin(); itrow != coordsInBaseFrame.end(); itrow++, row++)
    {
      //Each tracker -> a column
        int col = 0;
        for(vector<PersonModel>::iterator itcolumn = personList.begin(); itcolumn != personList.end(); itcolumn++, col++)
        {

            //Calculate the distance

            Point3d trackerPos2d = (*itcolumn).getPositionEstimate();
            Mat trackerColorHist = (*itcolumn).getBvtHistogram();
            Mat detectionColorHist = colorFeaturesList.at(row);


            detectionColorHist.convertTo(detectionColorHist, CV_32F);
            trackerColorHist.convertTo(trackerColorHist, CV_32F);

            Point3d trackerPos;

            trackerPos.x = trackerPos2d.x;
            trackerPos.y = trackerPos2d.y;
            trackerPos.z = 0;

            Point3d detectionPos = (*itrow);
            detectionPos.z = 0;

            double dist = norm(trackerPos-detectionPos);

            double colorDist;

            colorDist = compareHist(detectionColorHist, trackerColorHist, CV_COMP_CORREL);
            if(dist < associatingDistance)
            {
                if(colorDist < 0.3)
                  distMatrixIn[row+col*nDetections] = 1000; //Colors are too different. It cant be the same person...
                else
                    distMatrixIn[row+col*nDetections] = (1+dist)*pow(1.1-colorDist, 3);
            }
            else
            {
                double best = 1000;
                for(int i = 1; i<median_window; i++)
                {
                    Point3d hist((*itcolumn).positionHistory[i].x, (*itcolumn).positionHistory[i].y, 0);
                    double distHist = norm(hist-detectionPos);
                    if(distHist < 1.8)
                        if(distHist < best)
                            best = distHist;
                }

                distMatrixIn[row+col*nDetections] = (1+best)*pow(1.1-colorDist, 3);
            }
        }
    }


    assignmentoptimal(assignment, cost, distMatrixIn, nDetections, nTrackers);


    //assignment vector positions represents the detections and the value in each position represents the assigned tracker
    //if there is no possible association, then the value is -1
    //cost vector doesn't mean anything at this point...

    //for each detection we update it's tracker with the position

    for(int i=0; i<nDetections; i++)
    {

        //If there is an associated tracker and the distance is less than 2 meters we update the position
        if(assignment[i] != -1)          
        {
   //
            if(distMatrixIn[i+((int)assignment[i])*nDetections] < associatingDistance)
            {
            personList.at(assignment[i]).position.x = coordsInBaseFrame.at(i).x;
            personList.at(assignment[i]).position.y = coordsInBaseFrame.at(i).y;
            personList.at(assignment[i]).position.z = coordsInBaseFrame.at(i).z;

          //Bounding boxes...
              personList.at(assignment[i]).rect = rects.at(i);

          //Color histograms
              personList.at(assignment[i]).bvtHistogram = colorFeaturesList.at(i);
            }
        }
        else
        {
            //If there isn't, create a new tracker for each one - IF THERE IS NO OTHER TRACKER IN A 1.5m radius
            bool existsInRadius = false;

            Point3d coords = coordsInBaseFrame.at(i);
            coords.z = 0;

            for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
            {
                Point3d testPoint((*it).positionHistory[0].x, (*it).positionHistory[0].y, 0);

                if(norm(coords-testPoint) < associatingDistance)
                {
                  existsInRadius = true;
                  break;
                }
            }
            if(!existsInRadius)
            addPerson(coordsInBaseFrame.at(i), rects.at(i), colorFeaturesList.at(i));
        }
    }


    //free the memory!

    free(distMatrixIn);
    free(assignment);
    free(cost);

    }
    else
    {
        //We create a tracker for each detection, and if there are none, we do nothing! - can't create 2 trackers in the same 1.5m radius
        for(int i=0; i<nDetections; i++)
        {
            bool existsInRadius = false;

            Point3d coords = coordsInBaseFrame.at(i);
            coords.z = 0;

            for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
            {
                Point3d testPoint((*it).positionHistory[0].x, (*it).positionHistory[0].y, 0);
                if(norm(coords-testPoint) < associatingDistance)
                {
                  existsInRadius = true;
                  break;
                }
            }
            if(!existsInRadius)
            addPerson(coordsInBaseFrame.at(i), rects.at(i), colorFeaturesList.at(i));
        }

    }
    //for each associated tracker we update the detection. For everyone else there is Mastercard. Just kidding... trackers
    //wich have no detections associated will be updated with a -1000, -1000 detection

    updateList();
}

std::vector<PersonModel> PersonList::getValidTrackerPosition()
{

    std::vector<PersonModel> validTrackers;

    for(std::vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
    {
        Point3d med;
        med = (*it).medianFilter();

        if(med.x != -1000 and med.y != -1000)
            validTrackers.push_back(*it);
    }

    return validTrackers;

}
