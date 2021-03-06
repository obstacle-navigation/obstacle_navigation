/* March 24, 2013
 * A ton of stuff was worked on today. To list it all:
 * - Multiple publishers and subscribers were created.
 * - The nodes that are subscribed to are
 *    - /camera/rgb/image_raw
 *    - /camera/rgb/image_color
 *    - /camera/rgb/image_mono
 *    - /camera/depth/image
 * - These publishers simply apply operations to images with default parameters
 *    - Operations: canny, dilate, erode, sobel, color threshold, depth threshold
 */

#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>

//to remap a subscriber to a different topic, type:
//$ rosrun package_name node_name initial_topic:=remapped_topic

image_transport::Publisher defaultPub;
image_transport::Publisher cannyPub;
image_transport::Publisher dilatePub;
image_transport::Publisher erodePub;
image_transport::Publisher sobelPub;
image_transport::Publisher colorThresholdPub;
image_transport::Publisher depthThresholdPub;

//canny feature detection, uses gray image
void getCannyImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  // Get an edge image - here we use the canny edge detection algorithm to get the edges
  double threshold1 = 20;
  double threshold2 = 50;
  int apertureSize = 3;

  // The smallest of threshold1 and threshold2 is used for edge linking, 
  // the largest - to find initial segments of strong edges.  Play around 
  // with these numbers to get desired result, and/or pre-process the 
  // image, e.g. clean up, sharpen, blur).
  ROS_INFO("Canny applied");
  cv::Canny(inputImage, outputImage, threshold1, threshold2, apertureSize);
}

//color threshold transform, uses gray image
void getColorThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
 double thresh = 128;
 double maxVal = 255;
 int thresholdType = CV_THRESH_BINARY;

 ROS_INFO("Color Threshold applied");
 cv::threshold(inputImage, outputImage, thresh, maxVal, thresholdType);
}

//depth threshold transform, uses depth image
void getDepthThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
 double thresh = 1;
 double maxVal = 255;
 int thresholdType = CV_THRESH_BINARY_INV;

 ROS_INFO("Depth Threshold applied");
 cv::threshold(inputImage, outputImage, thresh, maxVal, thresholdType);
}

//dilate filter, uses color image
void getDilateImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  cv::Mat element(7, 7, CV_8U);
  cv::Point anchor(-1,-1);
  int iterations = 3;

  ROS_INFO("Dilate applied");
  cv::dilate(inputImage, outputImage, element, anchor, iterations);
}

//erode filter, uses color image
void getErodeImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  cv::Mat element(7, 7, CV_8U);
  cv::Point anchor(-1,-1);
  int iterations = 3;

  ROS_INFO("Erode applied");
  cv::erode(inputImage, outputImage, element, anchor, iterations);
}

//sobel filter, uses gray image
void getSobelImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  int ddepth = CV_8U; // output image is 8 bit unsigned
  int xorder = 1; // 1st derivative in x
  int yorder = 1; // 1st derivative in y
  int ksize = 5; // filter size is 5

  ROS_INFO("Sobel applied");
  cv::Sobel(inputImage, outputImage, ddepth, xorder, yorder, ksize);
}

//unfinished GrabCut
void getGrabCutImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  cv::Rect rectangle(inputImage.cols / 2 - 10, inputImage.rows / 2 - 10, inputImage.cols / 2 + 10, inputImage.rows / 2 + 10);
  cv::Mat bgModel, fgModel;

  ROS_INFO("GRABCUT APPLIED!!");
  cv::grabCut(inputImage, outputImage, rectangle, bgModel, fgModel, 1, cv::GC_INIT_WITH_RECT);
  
  // Generate output image
  cv::Mat foreground(inputImage.size(),CV_8UC3,cv::Scalar(255,255,255));
  inputImage.copyTo(foreground,outputImage);
  outputImage = outputImage&1;
  foreground = cv::Mat(inputImage.size(),CV_8UC3,cv::Scalar(255,255,255));
  inputImage.copyTo(foreground,outputImage); // bg pixels not copied

  cv::Point p1(inputImage.rows / 2 - 10, inputImage.cols / 2 - 10);
  cv::Point p2(inputImage.rows / 2 + 10, inputImage.cols / 2 + 10);
  cv::rectangle(outputImage, p1, p2, CV_RGB(255,0,0),1,8,0);
}

//callback method for operations that require a color image
void colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  cv_bridge::CvImagePtr bridge_image = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat cvImage = bridge_image->image;
  cv::Mat dilateOut, erodeOut;

  //set cv_bridge image to output image, encoding
  //bridge_image->image = outputImage;
  //bridge_image->encoding = "mono8";

  //set cv_bridge image matrix to output and publish
  getDilateImage(cvImage, dilateOut);
  bridge_image->image = dilateOut;
  dilatePub.publish(bridge_image->toImageMsg());

  getErodeImage(cvImage, erodeOut);
  bridge_image->image = erodeOut;
  erodePub.publish(bridge_image->toImageMsg());

  //bridge_image->image = cvOutput;
  //Publish image
  //sensor_msgs::ImagePtr outImage = bridge_image->toImageMsg();
  //pub.publish(outImage);
  //ROS_INFO("Hello");
}

//callback method for operations that require a gray image
void grayCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  cv_bridge::CvImagePtr bridge_image = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat cvImage = bridge_image->image;
  cv::Mat sobelOut, colorThresholdOut, cannyOut;

  //set cv_bridge image matrix to output and publish
  getSobelImage(cvImage, sobelOut);
  bridge_image->image = sobelOut;
  sobelPub.publish(bridge_image->toImageMsg());

  getColorThresholdImage(cvImage, colorThresholdOut);
  bridge_image->image = colorThresholdOut;
  colorThresholdPub.publish(bridge_image->toImageMsg());

  getCannyImage(cvImage, cannyOut);
  bridge_image->image = cannyOut;
  cannyPub.publish(bridge_image->toImageMsg());
}

//callback method for operations that require a depth image
void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  cv_bridge::CvImagePtr bridge_image = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat cvImage = bridge_image->image;
  cv::Mat depthThresholdOut;
  
  //set cv_bridge image matrix to output and publish
  getDepthThresholdImage(cvImage, depthThresholdOut);
  bridge_image->image = depthThresholdOut;
  depthThresholdPub.publish(bridge_image->toImageMsg());
}

//default callback method, copies input topic
void defaultCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  cv_bridge::CvImagePtr bridge_image = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat cvImage = bridge_image->image;
  cv::Mat defaultOut;

  ROS_INFO("Default applied?");
  defaultPub.publish(bridge_image->toImageMsg());
}


int main( int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "opencv");
  ros::NodeHandle n;

  //Set up image subscribers
  image_transport::ImageTransport it(n);
  image_transport::Subscriber defaultSub = it.subscribe("/camera/rgb/image_raw", 1, defaultCallback);
  image_transport::Subscriber colorSub = it.subscribe("/camera/rgb/image_color", 1, colorCallback);
  image_transport::Subscriber graySub = it.subscribe("/camera/rgb/image_mono", 1, grayCallback);
  image_transport::Subscriber depthSub = it.subscribe("/camera/depth/image", 1, depthCallback);
  //Set up image publishers
  defaultPub = it.advertise("/proj1/opencv/default",1);
  cannyPub = it.advertise("proj1/opencv/canny",1);
  dilatePub = it.advertise("proj1/opencv/dilate",1);
  erodePub = it.advertise("proj1/opencv/erode",1);
  sobelPub = it.advertise("proj1/opencv/sobel",1);
  colorThresholdPub = it.advertise("proj1/opencv/threshold/color",1);
  depthThresholdPub = it.advertise("proj1/opencv/threshold/depth",1);
  
  //Transfer control to ROS
  ros::spin();

  return 0;

}
