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


//Not used yet...
void PersonModel::updateVelocityArray(Point3d detectedPosition)
{
    for(int i=0; i < 5; i++)
        velocity[5-i] = velocity[5-(i+1)];

    if(detectedPosition.x != -1000 && detectedPosition.y != -1000)
    {
      velocity[0].x = (detectedPosition.x - position.x)/delta_t;
      velocity[0].y = (detectedPosition.y - position.y)/delta_t;
    }
     //If there is no suitabled detection we don't update it
    //This must be better thinked...
}


Point2d PersonModel::getPositionEstimate()
{

    //For now just return the last detection...
    return positionHistory[0];

}


void PersonModel::updateModel()
{

 //Maybe applying a median filter to the velocity gives good predictive results

  //updateVelocityArray(detectedPosition); - not used yet

  for(int i=0; i < 4; i++)
      positionHistory[4-i] = positionHistory[4-(i+1)];

  positionHistory[0] = position;

  position.x = -1000;
  position.y = -1000;


  for(int i=0; i < 4; i++)
      rectHistory[4-i] = rectHistory[4-(i+1)];

  rectHistory[0] = rect;

  //velocity = median...


}

PersonModel::PersonModel(Point3d detectedPosition, cv::Rect_<int> bb, int id)
{
    for(int i=0; i < 5; i++)
        velocity[i] = Point2d(0, 0);

  position.x = detectedPosition.x;
  position.y = detectedPosition.y;

  rect = bb;
  rectHistory[0] = bb;

  positionHistory[0].x = detectedPosition.x;
  positionHistory[0].y = detectedPosition.y;

  for(int i=1; i < 5; i++)
  {
      positionHistory[i].x =-1000;
      positionHistory[i].y = -1000;
  }

  lockedOnce = false;

  this->id = id;
  noDetection = 0;

}

Point3d PersonModel::getNearestPoint(vector<cv::Point3d> coordsInBaseFrame, Point2d estimation)
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
        if(dist < distance && dist < 2)
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

    double x[5];
    double y[5];

    for(int i = 0; i<5; i++)
    {
        x[i] = positionHistory[i].x;
        y[i] = positionHistory[i].y;
    }

    vector<double> x_vect(x, x + sizeof(x)/sizeof(x[0]));
    vector<double> y_vect(y, y + sizeof(y)/sizeof(y[0]));

    std::sort(x_vect.begin(), x_vect.begin() + 5);
    std::sort(y_vect.begin(), y_vect.begin() + 5);


    Point3d medianPoint(x_vect.at(2), y_vect.at(2), 0);

    return medianPoint;

}

PersonModel::~PersonModel()
{

}

PersonList::PersonList()
{
  //This will never get reseted. That will make sure that we have a new id for every new detection
  nPersons = 0;
}

void PersonList::updateList()
{
  //TODO

    for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end();)
    {

        if((*it).position.x != -1000 && (*it).position.y != -1000)
          (*it).noDetection = 0;
        else
        {
          (*it).noDetection++;
        }

        (*it).updateModel();

        if(((*it).noDetection > 5 && (*it).lockedOnce==false) || ((*it).noDetection > 1000000 && (*it).lockedOnce==true))
        {
            it = personList.erase(it);
        }
        else
            it++;
    }

}

void PersonList::addPerson(Point3d pos, cv::Rect_<int> rect)
{

   PersonModel person(pos, rect, nPersons);
   personList.push_back(person);
   nPersons++;

}

void PersonList::associateData(vector<Point3d> coordsInBaseFrame, vector<cv::Rect_<int> > rects)
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

            Point2d trackerPos2d = (*itcolumn).getPositionEstimate();

            Point3d trackerPos;

            trackerPos.x = trackerPos2d.x;
            trackerPos.y = trackerPos2d.y;
            trackerPos.z = 0;

            Point3d detectionPos = (*itrow);

            double dist = norm(trackerPos-detectionPos);
            if(dist < 2)
              distMatrixIn[row+col*nDetections] = dist;
            else
            {
                double best = 1000;
                for(int i = 1; i<5; i++)
                {
                    Point3d hist((*itcolumn).positionHistory[i].x, (*itcolumn).positionHistory[i].y, 0);

                    double distHist = norm(hist-detectionPos);
                    if(distHist < 1.8)
                        if(distHist < best)
                            best = distHist;
                }

                distMatrixIn[row+col*nDetections] = best;
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
            if(distMatrixIn[i+((int)assignment[i])*nDetections] < 2)
            {
            personList.at(assignment[i]).position.x = coordsInBaseFrame.at(i).x;
            personList.at(assignment[i]).position.y = coordsInBaseFrame.at(i).y;

          //Bounding boxes...
              personList.at(assignment[i]).rect = rects.at(i);
            }
        }
        else
        {
            //If there isn't, create a new tracker for each one - IF THERE IS NO OTHER TRACKER IN A 1.5m radius
            bool existsInRadius = false;

            for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
            {
                Point3d testPoint((*it).positionHistory[0].x, (*it).positionHistory[0].y, 0);
                if(norm(coordsInBaseFrame.at(i)-testPoint) < 1.5)
                {
                  existsInRadius = true;
                  break;
                }
            }
            if(!existsInRadius)
            addPerson(coordsInBaseFrame.at(i), rects.at(i));
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

            for(vector<PersonModel>::iterator it = personList.begin(); it != personList.end(); it++)
            {
                Point3d testPoint((*it).positionHistory[0].x, (*it).positionHistory[0].y, 0);
                if(norm(coordsInBaseFrame.at(i)-testPoint) < 1.5)
                {
                  existsInRadius = true;
                  break;
                }
            }
            if(!existsInRadius)
            addPerson(coordsInBaseFrame.at(i), rects.at(i));
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
