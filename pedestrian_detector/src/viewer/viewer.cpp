//ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class ImageConverter
{
  private:
  ros::NodeHandle nh;
  image_transport::ImageTransport *it;
  image_transport::Subscriber image_sub;
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat image(cv_ptr->image);
  
  cv::imshow("viewer", image);
  cv::waitKey(30);
  }
  
  
  // topic -> /pedestriandetector/image
  public:
  ImageConverter()
  {
    it = new image_transport::ImageTransport(nh);
    image_sub = it->subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "viewer");
  
  ImageConverter converter;
  
  ros::spin();
  return 0;
}
