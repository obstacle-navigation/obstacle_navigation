#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
//#include <vector>

//to remap a subscriber to a different topic, type:
//$ rosrun package_name node_name initial_topic:=remapped_topic

image_transport::Publisher defaultPub;
image_transport::Publisher cannyPub;
image_transport::Publisher dilatePub;
image_transport::Publisher erodePub;
image_transport::Publisher sobelPub;
image_transport::Publisher colorThresholdPub;
image_transport::Publisher depthThresholdPub;
image_transport::Publisher depthThresholdBWPub;

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

cv::Mat depthThreshold;

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
  //ROS_INFO("Canny applied");
  cv::Canny(inputImage, outputImage, threshold1, threshold2, apertureSize);

  //delete threshold1, threshold2, apertureSize;
}

//color threshold transform, uses gray image
void getColorThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
 double thresh = 128;
 double maxVal = 255;
 int thresholdType = CV_THRESH_BINARY;

 //ROS_INFO("Color Threshold applied");
 cv::threshold(inputImage, outputImage, thresh, maxVal, thresholdType);

  //delete thresh, maxVal, thresholdType;
}

void explore(cv::Mat& outputImage, int target, int row, int col) {
  if(row >=0 && row <= outputImage.rows && col >=0 && col <= outputImage.cols){
    float* rowp = outputImage.ptr<float>(row);
    //if(outputImage.at<float>(row,col) > 0){
      if(rowp[col] > 0){
      //std::cout << "target found at " << row << ", " << col << std::endl;
      totalArea++;
      //outputImage.at<float>(row,col) = 0;
      rowp[col] = 0;
      curblobArea++;
      if(curminXBound > col)
        curminXBound = col;
      if(curminYBound > row)
        curminYBound = row;
      if(curmaxXBound < col)
        curmaxXBound = col;
      if(curmaxYBound < row)
        curmaxYBound = row;
      //is this the problem? outputImage gets copied after each time explore is called
      explore(outputImage, target, row-1, col);
      explore(outputImage, target, row+1, col);
      explore(outputImage, target, row, col-1);
      explore(outputImage, target, row, col+1);
      /*int rn[4] = {1, -1, 0, 0};
      int cn[4] = {0, 0, -1, 1};
      for(int i=0;i<4;i++){
        cv::Mat temp = outputImage.clone();
        explore(temp, target, row+rn[i], col+cn[i]);      
      }*/

      /*explore(outputImage, target, row-1, col-1);
      explore(outputImage, target, row-1, col+1);
      explore(outputImage, target, row+1, col-1);
      explore(outputImage, target, row+1, col+1);*/
    }
  }
}

//this uses a separate 2D array of booleans instead of modifying outputImage
void explore2(int target, int row, int col, bool *checked) {
  if(row >=0 && row <= depthThreshold.rows && col >=0 && col <= depthThreshold.cols){
    float* rowp = depthThreshold.ptr<float>(row);
    //if(outputImage.at<float>(row,col) > 0){
      if(rowp[col] > 0 && checked[row*depthThreshold.rows+col]==0){
      //std::cout << "target found at " << row << ", " << col << std::endl;
      totalArea++;
      //outputImage.at<float>(row,col) = 0;
      curblobArea++;
      checked[row*depthThreshold.rows+col]=1;
      //std::cout << row << ", " << col << std::endl;
      if(curminXBound > col)
        curminXBound = col;
      if(curminYBound > row)
        curminYBound = row;
      if(curmaxXBound < col)
        curmaxXBound = col;
      if(curmaxYBound < row)
        curmaxYBound = row;
      //is this the problem? outputImage changes after each time explore is called
      explore2(target, row-1, col, checked);
      explore2(target, row+1, col, checked);
      explore2(target, row, col-1, checked);
      explore2(target, row, col+1, checked);
      /*int rn[4] = {1, -1, 0, 0};
      int cn[4] = {0, 0, -1, 1};
      for(int i=0;i<4;i++){
        cv::Mat temp = outputImage.clone();
        explore(temp, target, row+rn[i], col+cn[i]);      
      }*/

      /*explore(outputImage, target, row-1, col-1);
      explore(outputImage, target, row-1, col+1);
      explore(outputImage, target, row+1, col-1);
      explore(outputImage, target, row+1, col+1);*/
    }
  }
}

//what about shrinking the array? turn 4 pixel blocks into 1 "pixel"
void explore3(int target, int row, int col, bool *checked){
  if(row >=0 && row <= depthThreshold.rows && col >=0 && col <= depthThreshold.cols){
    float* rowp = depthThreshold.ptr<float>(row);
    //if(outputImage.at<float>(row,col) > 0){
      if(rowp[col] > 0 && checked[row*depthThreshold.rows+col]==0){
      //std::cout << "target found at " << row << ", " << col << std::endl;
      totalArea += 1;
      curblobArea += 4;
      checked[row*depthThreshold.rows+col]=1;
      if(curminXBound > col)
        curminXBound = col;
      if(curminYBound > row)
        curminYBound = row;
      if(curmaxXBound < col)
        curmaxXBound = col;
      if(curmaxYBound < row)
        curmaxYBound = row;
      explore3(target, row-2, col, checked);
      explore3(target, row+2, col, checked);
      explore3(target, row, col-2, checked);
      explore3(target, row, col+2, checked);
    }
  }
}

void explore(const cv::Mat& inputImage, cv::Mat& outputImage, int target) {
  blobArea = 0;
  minXBound = 1000;
  minYBound = 1000;
  maxXBound = 0;
  maxYBound = 0;
  totalArea = 0;
  int i = 0, j = 0;
  /*bool checked[depthThreshold.rows*depthThreshold.cols];
  bool *boolpointer = checked;*/
  /*for(i = 0; i < depthThreshold.rows*depthThreshold.cols; i++){
    if(boolpointer[i]!=false){
        std::cout<<boolpointer[i]<<" ";
        boolpointer[i] = false;
      }
  }*/
  /*bool *boolpointer = new bool[depthThreshold.rows*depthThreshold.cols];
  boolpointer = {false};*/
  bool *boolpointer;
  boolpointer = (bool*) calloc(depthThreshold.rows*depthThreshold.cols,sizeof(bool));
//Print and checking the blob area


 /* std::cout << "[total area = " << totalArea << "]" << std::endl;
  std::cout << "max blob = " << blobArea << ", (" << minXBound << ", " << minYBound << "), ";
  std::cout << "(" << maxXBound << ", " << maxYBound << ")" << std::endl;
  std::cout << "------------------------------------------------------------" << std::endl;*/
  
  /*int pixels = 0;
  for(int ii=0; i < inputImage.rows; ii++) {
    for(int jj=0; jj < inputImage.cols; jj++) {
      if(inputImage.at<float>(ii,jj) > 0)
        pixels++;
    }
  }

  std::cout << "------------------------------------------------------------" << std::endl;
  std::cout << "|" << pixels << "|" << std::endl;
  std::cout << "------------------------------------------------------------" << std::endl;*/
  
  for(i=0; i < inputImage.rows; i++) {
    //const float* rowp = inputImage.ptr<float>(i);
    for(j=0; j < inputImage.cols; j++) {
      if(inputImage.at<float>(i,j) > 0 && !boolpointer[depthThreshold.rows*i + j]) {
      //if(rowp[j] > 0) {
        //std::cout << "[(" << i << ", " << j << ")]" << std::endl;
        curblobArea = 0;
        curminXBound = 1000;
        curminYBound = 1000;
        curmaxXBound = 0;
        curmaxYBound = 0;

        //explore(outputImage, target, i, j);
        //explore2(target, i, j, boolpointer);
        explore3(target, i, j, boolpointer);
    
        /*pixels = 0;
        for(ii = 0; ii < inputImage.rows; ii++) {
          for(jj = 0; jj < inputImage.cols; jj++) {
            if(outputImage.at<float>(ii,jj) > 0)
              pixels++;
          }
        }
        std::cout << "pixels after: " << pixels << std::endl;
        if (curblobArea > 0) {
          std::cout << "[found a blob with area = " << curblobArea << " with top-left at ";
          std::cout << "(" << curminXBound << ", " << curminYBound << ")]" << std::endl;
        }*/
        if (curblobArea > blobArea) {
          blobArea = curblobArea;
          minXBound = curminXBound;
          minYBound = curminYBound;
          maxXBound = curmaxXBound;
          maxYBound = curmaxYBound;
        }
      }
    }
    
  }

  std::cout << "[total area = " << totalArea << "]" << std::endl;
  std::cout << "max blob = " << blobArea << ", (" << minXBound << ", " << minYBound << "), ";
  std::cout << "(" << maxXBound << ", " << maxYBound << ")" << std::endl;
  std::cout << "------------------------------------------------------------" << std::endl;
  
  free(boolpointer);
  //delete[] boolpointer;
  //delete jj, ii, i, j, pixels;
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

//dilate filter, uses color image
void getDilateImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  cv::Mat element(7, 7, CV_8U);
  cv::Point anchor(-1,-1);
  int iterations = 3;

  //ROS_INFO("Dilate applied");
  cv::dilate(inputImage, outputImage, element, anchor, iterations);

  //delete iterations;
}

//erode filter, uses color image
void getErodeImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  cv::Mat element(7, 7, CV_8U);
  cv::Point anchor(-1,-1);
  int iterations = 3;

  //ROS_INFO("Erode applied");
  cv::erode(inputImage, outputImage, element, anchor, iterations);

  //delete iterations;
}

//sobel filter, uses gray image
void getSobelImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  int ddepth = CV_8U; // output image is 8 bit unsigned
  int xorder = 1; // 1st derivative in x
  int yorder = 1; // 1st derivative in y
  int ksize = 5; // filter size is 5

  //ROS_INFO("Sobel applied");
  cv::Sobel(inputImage, outputImage, ddepth, xorder, yorder, ksize);

  //delete ddepth, xorder, yorder, ksize;
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
  //std::cout << "bridge_image created" << std::endl;
  cv::Mat cvImage = bridge_image->image;
  //std::cout << "cvImage created" << std::endl;
  cv::Mat depthThresholdOut;
  
  //set cv_bridge image matrix to output and publish
  getDepthThresholdImage(cvImage, depthThresholdOut);
  //std::cout << "got depthThresholdImage" << std::endl;
  bridge_image->image = depthThresholdOut;
  //std::cout << "bridge_image->image = depthThresholdOut" << std::endl;
  depthThresholdPub.publish(bridge_image->toImageMsg());
  //printImagePixels(depthThresholdOut, "output depth threshold");
  //std::cout << depthThresholdOut << "\n*****\n" << std::endl;
  
  //cv::Mat temp;
  //depthThresholdOut.copyTo(temp);
  
  depthThreshold = depthThresholdOut;
  //printImageData(depthThresholdOut, "depthThesholdOut");
  explore(depthThresholdOut, depthThresholdOut, 255);
  
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

  //cvReleaseMat(&cvImage);
  //cvReleaseMat(&depthThresholdOut); 

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
  
  delete &bridge_image;
  delete &cvImage;
}

int main( int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "opencv");
  ros::NodeHandle n;

  //Set up image subscribers
  image_transport::ImageTransport it(n);
  image_transport::Subscriber defaultSub = it.subscribe("/camera/rgb/image_raw", 1, defaultCallback);
  //image_transport::Subscriber colorSub = it.subscribe("/camera/rgb/image_color", 1, colorCallback);
  image_transport::Subscriber graySub = it.subscribe("/camera/rgb/image_mono", 1, grayCallback);
  image_transport::Subscriber depthSub = it.subscribe("/camera/depth/image", 1, depthCallback);
  //Set up image publishers
  defaultPub = it.advertise("/obsnav/opencv/default",1);
  cannyPub = it.advertise("/obsnav/opencv/canny",1);
  dilatePub = it.advertise("/obsnav/opencv/dilate",1);
  erodePub = it.advertise("/obsnav/opencv/erode",1);
  sobelPub = it.advertise("/obsnav/opencv/sobel",1);
  colorThresholdPub = it.advertise("/obsnav/opencv/threshold/color",1);
  //depthThresholdPub = it.advertise("/obsnav/opencv/threshold/depth",1);
  
  ROS_INFO("Started");
  //Transfer control to ROS
  ros::spin();

  return 0;

}
