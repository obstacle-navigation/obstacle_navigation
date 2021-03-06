#include <ros/ros.h>
#include <ros/time.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <obstacle_navigation/Blob.h>
#include <obstacle_navigation/Blobs.h>

//to remap a subscriber to a different topic, type:
//$ rosrun package_name node_name initial_topic:=remapped_topic

const int slice_count = 4;
const double slice_size = 1.0;

image_transport::Publisher depthThresholdPub;
ros::Publisher blobPub;

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

obstacle_navigation::Blobs blob_message;
int blobindex = 0;
int blobs[slice_count][7];

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
}

//what about shrinking the array? turn 4 pixel blocks into 1 "pixel"
void explore3(int target, int row, int col, bool *checked){
  if(row >=0 && row <= depthThreshold.rows && col >=0 && col <= depthThreshold.cols){
    float* rowp = depthThreshold.ptr<float>(row);
      if(rowp[col] > 0 && checked[row*depthThreshold.rows+col]==0){
      totalArea += 1; //something is wrong with this
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

void explore(const cv::Mat& inputImage, int target) {
  depthThreshold = inputImage;
  blobArea = 0;
  minXBound = 1000;
  minYBound = 1000;
  maxXBound = 0;
  maxYBound = 0;
  totalArea = 0;
  int i = 0, j = 0;

  bool *boolpointer;
  boolpointer = (bool*) calloc(depthThreshold.rows*depthThreshold.cols,sizeof(bool));
  
  for(i=0; i < inputImage.rows; i++) {
    for(j=0; j < inputImage.cols; j++) {
      if(inputImage.at<float>(i,j) > 0 && !boolpointer[depthThreshold.rows*i + j]) {
        curblobArea = 0;
        curminXBound = 1000;
        curminYBound = 1000;
        curmaxXBound = 0;
        curmaxYBound = 0;

        explore3(target, i, j, boolpointer);

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

  /*std::cout << "[total area = " << totalArea << "]" << std::endl;
  std::cout << "max blob = " << blobArea << ", (" << minXBound << ", " << minYBound << "), ";
  std::cout << "(" << maxXBound << ", " << maxYBound << ")" << std::endl;
  std::cout << "------------------------------------------------------------" << std::endl;*/

  /*blobs[blobindex][0] = blobArea;			//area
  blobs[blobindex][1] = (maxXBound + minXBound)/2;	//center x
  blobs[blobindex][2] = (maxYBound + minYBound)/2;	//center y
  blobs[blobindex][3] = minXBound;			//left
  blobs[blobindex][4] = maxXBound;			//right
  blobs[blobindex][5] = minYBound;			//top
  blobs[blobindex][6] = maxYBound;			//bottom*/

  blob_message.blobs[blobindex].area = blobArea;
  blob_message.blobs[blobindex].x = (maxXBound + minXBound)/2;
  blob_message.blobs[blobindex].y = (maxYBound + minYBound)/2;
  blob_message.blobs[blobindex].left = minXBound;
  blob_message.blobs[blobindex].right = maxXBound;
  blob_message.blobs[blobindex].top = minYBound;
  blob_message.blobs[blobindex].bottom = maxYBound;
  //blobindex is set at depthCallback loop
  
  free(boolpointer);
}

//depth threshold transform, uses depth image
void getDepthThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage, double min, double max) {
 double maxVal = 255;
 int thresholdType = CV_THRESH_TOZERO_INV;

 //ROS_INFO("Depth Threshold applied");
 //first set all values further than the max to 0, leave values less than the same
 cv::threshold(inputImage, outputImage, max, maxVal, thresholdType);

 thresholdType = CV_THRESH_BINARY;
 cv::Mat temp = outputImage.clone();
 //second, set all values further than min to 255, set those less than to 0
 cv::threshold(temp, outputImage, min, maxVal, thresholdType);
}

void printblob(int index){
  printf("index area   x   y    left  right top   bottom\n");
  printf("%-5d %-6d %-3d %-3d %-5d %-5d %-5d %-5d\n",index,blobs[blobindex][0],blobs[blobindex][1],blobs[blobindex][2],blobs[blobindex][3],blobs[blobindex][4],blobs[blobindex][5],blobs[blobindex][6]);
}

//callback method for operations that require a depth image
void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  cv_bridge::CvImagePtr bridge_image = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat cvImage = bridge_image->image;
  cv::Mat depthThresholdOut;
  
  blob_message.blobs.resize(slice_count);
  for(int i=0; i<slice_count; i++){
    blobindex = i; 
    getDepthThresholdImage(cvImage, depthThresholdOut, i*slice_size, (i+1)*slice_size);
    blob_message.blobs[blobindex].mindepth = i*slice_size;
    blob_message.blobs[blobindex].maxdepth = (i+1)*slice_size;
    if(i==0){
        //publish a certain slice, i
        bridge_image->image = depthThresholdOut;
        depthThresholdPub.publish(bridge_image->toImageMsg());
    }
    explore(depthThresholdOut, 255);
    //printblob(blobindex);
  }
  //printf("------------------------------------------------\n");
  blob_message.header.stamp = ros::Time::now();
  blob_message.blob_count = slice_count;
  blobPub.publish(blob_message);
  //set cv_bridge image matrix to output and publish
}


int main( int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "opencv");
  ros::NodeHandle n;

  //blob publisher
  blobPub = n.advertise<obstacle_navigation::Blobs>("/obsnav/depth_blobs/blobs",1);
  //Set up image subscribers
  image_transport::ImageTransport it(n);
  image_transport::Subscriber depthSub = it.subscribe("/camera/depth/image", 1, depthCallback);
  //Set up image publishers
  depthThresholdPub = it.advertise("/obsnav/depth_blobs/threshold",1);
  
  ROS_INFO("depth_blobs Started");
  //Transfer control to ROS
  ros::spin();

  return 0;

}
