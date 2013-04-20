#include <ros/ros.h>
#include <image_transport/image_transport.h>

image_transport::Publisher trans_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("hello");
  trans_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transporter");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/camera/depth/image", 1, imageCallback);
  trans_pub = it.advertise("/transporter", 1);
  ros::spin();
}
