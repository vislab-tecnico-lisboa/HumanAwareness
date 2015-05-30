#include "../include/cameraModel.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

cameraModel::cameraModel(string configFile)
{
  FileStorage fs(configFile, FileStorage::READ);
  if(!fs.isOpened())
    {
      cout << "Couldn't open camera config file." << endl;
      exit(-4);
    }

  fs["camera_matrix"] >> K_;
  fs["distortion_coefficients"] >> distCoefs_;
//  fs["projection_matrix"] >> projectionMat_;

  invert(K_, invertedK);
  invertedK.convertTo(invertedK, CV_32FC1);

  //Just initializing pose of the camera relative to the base of the robot...
  float pose_array[] = {0, 1, 0, 0, 0, 0, -1, 800, -1, 0, 0, 0};
  pose = Mat(3, 4, CV_32FC1, pose_array).clone();

  //If we consider the point to be on the floor, then relatively to the base frame they will have z=0
  //Therefore the 3rd column of the [R|t] matrix is 0, and we have and homography that we can invert.
  float homography_array[] = {0, 1, 0, 0, 0, 800, -1, 0, 0};
  homography = Mat(3, 3, CV_32FC1, homography_array).clone();


  invert(homography, invertedHomography);
  invertedHomography.convertTo(invertedHomography, CV_32FC1);

}

vector<Point3d> cameraModel::calculatePointsOnBaseFrame(Mat imagePoints)
{

  //Transform the points to homogeneous coordinates


  Mat transposedPoints;

  transpose(imagePoints, transposedPoints);
  Mat ones = Mat::ones(1, transposedPoints.size().width, CV_32FC1);


  transposedPoints.convertTo(transposedPoints, CV_32FC1);
  transposedPoints.push_back(ones);

  Mat &homogeneousPoints = transposedPoints;

  //First normalize the points
  // K^(-1)*x_cam = [R|t]*p


  Mat normalizedPoints = invertedK*homogeneousPoints;


  //Finally we get the points on the base frame in homogeneous coordinates
  // p = H^-1 * (K^-1 * x_cam)

  Mat homogeneousP = invertedHomography*normalizedPoints;

  //Now we just get the x, y from the homogeneous coordinates and set z to 0
  /*
            [p1x p2x p3x ... pnx]
  p_tilde=  [p1y p2y p3y ... pny]
            [l_1 l_2 l_3 ... l_n]
  */

  //x = pix/l_i
  //y = piy/l_i

  vector<Point3d> basePoints;

   for(int i = 0; i< homogeneousP.size().width; i++)
     {
       Point3d point;
       point.x = homogeneousP.at<float>(0, i)/homogeneousP.at<float>(2, i);
       point.y = homogeneousP.at<float>(1, i)/homogeneousP.at<float>(2, i);
       point.z = 0;
       basePoints.push_back(point);
     }

   return basePoints;

}

Mat cameraModel::getK()
{
  return K_;
}


Mat cameraModel::getDistCoefs()
{
  return distCoefs_;
}


Mat cameraModel::getProjectionMat()
{
  return projectionMat_;
}
