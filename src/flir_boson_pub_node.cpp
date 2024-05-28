#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "bosonUSB.h"

void publish_thermal_img(ros::Publisher& pub, const cv::Mat& thermal_img, const std::string& encoding)
{
  sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
  msg->header.stamp = ros::Time::now();
  msg->height = thermal_img.rows;
  msg->width = thermal_img.cols;
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = thermal_img.step;
  msg->data.assign(thermal_img.datastart, thermal_img.dataend);
  pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boson_pub");
  ros::NodeHandle nh;
  ros::Publisher thermal_pub = nh.advertise<sensor_msgs::Image>("boson/image_raw", 1);

  Boson boson;
  cv::Mat thermal_img;

//   if (!boson.initialize()) {
//     ROS_ERROR("Failed to initialize BosonUSB.");
//     return -1;
//   }

  ros::Rate loop_rate(60); // Adjust the rate as needed

  while (ros::ok()) {
    if (boson.read_frame(thermal_img)) {
      publish_thermal_img(thermal_pub, thermal_img, "mono8"); // Publishing thermal16_linear
      // Alternatively, publish thermal_rgb if needed:
      // publish_thermal_img(thermal_pub, thermal_rgb, "bgr8");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}