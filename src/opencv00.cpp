/*
* March 23, 2013
* This node takes in an sensor_msg/Image input topic and
* will convert it to cv_bridge. Next it will get the image
* from cv_bridge and go back to a sensor_msg/Image and
* publish it.
*/

#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

image_transport::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImageConstPtr bridge_image = cv_bridge::toCvShare(msg, "bgr8");
  //cv::Mat cvImage = bridge_image->image;
  //cv::Mat cvOutput; 

  
  //Publish image
  sensor_msgs::ImagePtr outImage = bridge_image->toImageMsg();
  pub.publish(outImage);

}

int main( int argc, char** argv)
{

  //Init ROS
  ros::init(argc, argv, "opencv");
  ros::NodeHandle n;


  //Set up image publish and subscribe
  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, callback);

  pub = it.advertise("/proj1/opencv",1);

  //Transfer control to ROS
  ros::spin();

  return 0;

}
