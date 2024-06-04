#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "bosonUSB.h"

void publish_thermal_img(ros::Publisher& pub, const cv::Mat& thermal_img, const int dtype)
{
  sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
  msg->header.stamp = ros::Time::now();
  msg->height = thermal_img.rows;
  msg->width = thermal_img.cols;
  msg->encoding = (dtype == RAW16) ? "mono16" : (dtype == YUV) ? "mono8" : "";
  msg->is_bigendian = false;
  msg->step = thermal_img.step;
  msg->data.assign(thermal_img.datastart, thermal_img.dataend);
  pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flir_boson_pub");
  ros::NodeHandle nh;

  std::string port;
  int dtype;

  // Get required parameters
  nh.param<std::string>("port", port, "/dev/video0");
  nh.param<int>("dtype", dtype, 1); // either 0 (agc8) or 1 (raw16)

  ros::Publisher thermal_pub = nh.advertise<sensor_msgs::Image>("boson/image_raw", 1);

  std::cout << "Port: " << port << std::endl;
  std::cout << "Dtype: " << dtype << std::endl;

  Boson boson(port, dtype);
  cv::Mat thermal_img;

  // if (!boson.initialize()) {
  //   ROS_ERROR("Failed to initialize BosonUSB.");
  //   return -1;
  // }

  ros::Rate loop_rate(60); // Adjust the rate as needed

  while (ros::ok()) {
    if (boson.read_frame(thermal_img)) {
      publish_thermal_img(thermal_pub, thermal_img, dtype);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}