#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <cmath>
//#include <vector>

//to remap a subscriber to a different topic, type:
//$ rosrun package_name node_name initial_topic:=remapped_topic

image_transport::Publisher defaultPub;
image_transport::Publisher depthThresholdPub;
image_transport::Publisher depthThresholdBWPub;
image_transport::Publisher motionDetectPub;

int blobArea;
int minXBound;
int minYBound;
int maxXBound;
int maxYBound;

int curblobArea;
int curminXBound;
int curminYBound;
int curmaxXBound;
int curmaxYBound;

int totalArea;

cv_bridge::CvImagePtr bridge_image_depth;
cv::Mat cvImage_depth;
cv::Mat depthThresholdOut;
cv::Mat lastImage;

void printImageData(const cv::Mat& input, std::string description){
  std::cout << description << std::endl;
  std::cout << "matrix type: " << input.type() << std::endl;
  std::cout << "matrix depth: " << input.depth() << std::endl;
  std::cout << "channels: " << input.channels() << std::endl;
  std::cout << "size: " << input.size() << std::endl;
  std::cout << "total elem: " << input.total() << std::endl;
  std::cout << std::endl;
}

void printImagePixels(const cv::Mat& input, std::string description){
  std::cout << description << std::endl;
  int i = 0, j = 0;
  for(; i<input.rows; i++){
    for(; j<input.cols; j++){
      std::cout << input.at<float>(i,j) << " ";
    }
    std::cout << std::endl;
  }

  //delete i, j;
}

//depth threshold transform, uses depth image
void getDepthThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
 double thresh = 1.85;
 double maxVal = 255;
 int thresholdType = CV_THRESH_BINARY_INV;

 //ROS_INFO("Depth Threshold applied");
 cv::threshold(inputImage, outputImage, thresh, maxVal, thresholdType);

  //delete thresh, maxVal, thresholdType;
}

void getDepthThresholdBWImage(const cv::Mat& inputImage, cv::Mat& outputImage){
  /*cv::Mat in[] = {inputImage, inputImage,inputImage};
  cv::merge(in, 3, outputImage);*/

  //printImageData(outputImage,"output");

  //cv::Mat cvt;
  //cv::cvtColor(inputImage, cvt, CV_GRAY2BGR);
  //cvt = outputImage;

  //int from_to[] = { 0,0, 1,1, 2,2 };
  //cv::mixChannels(&cvt, 1, &outputImage, 1, from_to, 3);

  for(int i=0;i<inputImage.rows;i++){
    for(int j=0;j<inputImage.cols;j++){
      outputImage.at<cv::Vec3d>(i,j)[0] = (int)(inputImage.at<float>(i,j));
      outputImage.at<cv::Vec3d>(i,j)[1] = (int)(inputImage.at<float>(i,j));
      outputImage.at<cv::Vec3d>(i,j)[2] = (int)(inputImage.at<float>(i,j));
    }
  }
}

void getImageDifference(const cv::Mat& image1, cv::Mat& image2){
  std::cout << "trying to get image difference..." << std::endl;
  for(int i=0;i<image1.rows;i++){
    for(int j=0;j<image1.cols;j++){
      image2.at<float>(i,j) = std::abs(image1.at<float>(i,j) - image2.at<float>(i,j));
    }
  }
}

//callback method for operations that require a depth image
void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  bridge_image_depth = cv_bridge::toCvCopy(msg, msg->encoding);
  std::cout << "bridge_image created" << std::endl;
  cvImage_depth = bridge_image_depth->image;
  std::cout << "cvImage created" << std::endl;
  
  //set cv_bridge image matrix to output and publish
  getDepthThresholdImage(cvImage_depth, depthThresholdOut);
  std::cout << "got depthThresholdImage" << std::endl;
  bridge_image_depth->image = depthThresholdOut;
  std::cout << "bridge_image->image = depthThresholdOut" << std::endl;
  depthThresholdPub.publish(bridge_image_depth->toImageMsg());
  //printImagePixels(depthThresholdOut, "output depth threshold");
  //std::cout << depthThresholdOut << "\n*****\n" << std::endl;
  
  //cv::Mat temp;
  //depthThresholdOut.copyTo(temp);
  
  //printImageData(depthThresholdOut, "depthThesholdOut");
  
  if (lastImage.rows > 0) {
    getImageDifference(cvImage_depth, lastImage);
    bridge_image_depth->image = lastImage;
    motionDetectPub.publish(bridge_image_depth->toImageMsg());
  } else {
    std::cout << "lastImage hasn't been created yet" << std::endl;
  }
  
  lastImage = cvImage_depth;
  
/*
  cv::Mat depthThresholdBWOut;
  //getDepthThresholdBWImage(depthThresholdOut, depthThresholdBWOut);
  //printImageData(depthThresholdBWOut,"depthBW");
  //depthThresholdBWOut = depthThresholdOut;
  bridge_image->image = depthThresholdBWOut;
  depthThresholdBWPub.publish(bridge_image->toImageMsg());
*/

  //delete &depthThresholdOut;
  //delete &cvImage;

/*cvReleaseMat(&cvImage);
cvReleaseMat(&depthThresholdOut); 
*/
}

//default callback method, copies input topic
void defaultCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  cv_bridge::CvImagePtr bridge_image = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat cvImage = bridge_image->image;
  
  // ^ makes a copy, so I think it's allocating memory and needs to be deleted
  // cv::Mat defaultOut;

  //std::cout << msg->encoding << std::endl;
  //ROS_INFO("Default applied?");
  defaultPub.publish(bridge_image->toImageMsg());
  
  //delete &bridge_image;
  //delete &cvImage;
}

int main( int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "opencv");
  ros::NodeHandle n;

  //Set up image subscribers
  image_transport::ImageTransport it(n);
  image_transport::Subscriber depthSub = it.subscribe("/camera/depth/image", 1, depthCallback);
  //Set up image publishers
  depthThresholdPub = it.advertise("obsnav/threshold/depth",1);
  depthThresholdBWPub = it.advertise("obsnav/threshold/depth/bw",1);
  motionDetectPub = it.advertise("obsnav/motiondetect",1);
  
  ROS_INFO("Started");
  //Transfer control to ROS
  ros::spin();

  return 0;

}
