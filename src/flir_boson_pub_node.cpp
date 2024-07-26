#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include "bosonUSB.h"
extern "C" {
  #include "Client_API.h"
  #include "EnumTypes.h"
  #include "UART_Connector.h"
  #include "serialPortAdapter.h"
}

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
  ros::init(argc, argv, "flir_boson_pub_node");
  ros::NodeHandle nh("~");

  std::string port;
  int dtype;
  std::string ACM_port;
  std::string camera_info_path;
  int EXT_SYNC_MODE;
  int FFC_MODE;

  // Get required parameters
  nh.param<std::string>("port", port, "/dev/video0");
  nh.param<int>("dtype", dtype, 0); // either 0 (agc8) or 1 (raw16)
  nh.param<std::string>("ACM_port", ACM_port, "/dev/ttyACM0"); 
  nh.param<std::string>("camera_info_path", camera_info_path, "../config/boson.yaml");
  nh.param<int>("FFC_MODE", FFC_MODE, 1);
  nh.param<int>("EXT_SYNC_MODE", EXT_SYNC_MODE, 0);

  ROS_INFO("Port: %s", port.c_str());
  ROS_INFO("Dtype: %d", dtype);
  ROS_INFO("ACM port: %s", ACM_port.c_str());
  ROS_INFO("EXT_SYNC_MODE %d", EXT_SYNC_MODE);
  

  camera_info_manager::CameraInfoManager cam_info_manager(nh, "boson", "file://" + camera_info_path);
  sensor_msgs::CameraInfo camera_info = cam_info_manager.getCameraInfo();
  ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("boson/camera_info", 1);
  ros::Publisher thermal_pub = nh.advertise<sensor_msgs::Image>("boson/image_raw", 1);

  ROS_INFO("Starting FLIR Boson Node");


  char* acm_port_cstr = new char[ACM_port.length() + 1];
  std::strcpy(acm_port_cstr, ACM_port.c_str());
  int32_t port_num = FSLP_lookup_port_id(acm_port_cstr, strlen(acm_port_cstr));
  ROS_INFO("Port number: %d\n", port_num);

	FLR_RESULT result;
	result = Initialize(port_num, 921600); //COM6, 921600 baud (port_number=5 for COM6)
  ROS_INFO("Initialize: 0x%08X\n", result); 
  uint32_t camera_sn;
  result = bosonGetCameraSN(&camera_sn);
  ROS_INFO("Camera serial number: %d\n", camera_sn);

  // Set FFC mode
  e_FLR_BOSON_FFCMODE_E ffc_mode = static_cast<e_FLR_BOSON_FFCMODE_E>(FFC_MODE);
  FLR_BOSON_FFCMODE_E ffc_mode_result;
  FLR_RESULT ffc_set = bosonSetFFCMode(ffc_mode);
  result = bosonGetFFCMode(&ffc_mode_result);
  const char* ffc_result;
  if (ffc_mode_result == 0)
    ffc_result = "MANUAL_MODE";
  else if (ffc_mode_result == 1)
    ffc_result = "AUTO_MODE";
  else
    ffc_result = "Not valid";
  ROS_INFO("Camera FFC Mode result: %s\n", ffc_result);

  // Get AGC mode
  FLR_AGC_MODE_E agc_mode_result;
  result = agcGetMode(&agc_mode_result);
  ROS_INFO("Camera AGC result: %d\n", agc_mode_result);

  // Set external sync mode
  e_FLR_BOSON_EXT_SYNC_MODE_E sync_mode = static_cast<e_FLR_BOSON_EXT_SYNC_MODE_E>(EXT_SYNC_MODE);
  FLR_BOSON_EXT_SYNC_MODE_E sync_mode_result;
  FLR_RESULT sync_set = bosonSetExtSyncMode(sync_mode);
  result = bosonGetExtSyncMode(&sync_mode_result);
  const char* sync_result;
  if (sync_mode_result == 0)
    sync_result = "DISABLE_MODE";
  else if (sync_mode_result == 2)
    sync_result = "SLAVE_MODE";
  else
    sync_result = "Not valid";
  ROS_INFO("Camera SYNC mode result: %s\n", sync_result);

  FLR_BOSON_TIMESTAMPTYPE_E timeStampType = FLR_BOSON_FIRSTVALIDIMAGE;
  float timeStamp;

  Boson boson(port, dtype);
  cv::Mat thermal_img;

  ros::Rate loop_rate(10); // Adjust the rate as needed

  while (ros::ok()) {
    ros::spinOnce(); // Process incoming messages

    if (boson.read_frame(thermal_img)) {
      camera_info_pub.publish(camera_info);
      publish_thermal_img(thermal_pub, thermal_img, dtype);
    } 
    loop_rate.sleep();
    }


  // if (!boson.initialize()) {
  //   ROS_ERROR("Failed to initialize BosonUSB.");
  //   return -1;
  // }

  // ros::Rate loop_rate(30); // Adjust the rate as needed

  // while (ros::ok()) {
  //   if (boson.read_frame(thermal_img)) {
  //     // Timestamp
  //     FLR_RESULT result = bosonGetTimeStamp(timeStampType, &timeStamp);
  //     // ROS_INFO("timestamp: %f\n", timeStamp);
  //     camera_info_pub.publish(camera_info);
  //     publish_thermal_img(thermal_pub, thermal_img, dtype);

  //   }
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  ROS_INFO("Shutting down FLIR Boson Node");

  return 0;
}