/*
------------------------------------------------------------------------
-  FLIR Systems - Linux Boson  Capture & Recording                     -
------------------------------------------------------------------------
-  This code is using part of the explanations from this page          -
-  https://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/  -
-                                                                      -
-  and completed to be used with FLIR Boson cameras in 16 and 8 bits.  -
-  Internal AGC for 16bits mode is very basic, with just the intention -
-  of showing how to make that image displayable                       - 
------------------------------------------------------------------------

 BosonUSB [r/y/z/s/t/f] [0..9]
	r    : raw16 bits video input (default)
	y    : agc-8 bits video input
	z    : zoom mode to 640x480 (only applies to raw16 input)
        f<name> : record TIFFS in Folder <NAME>
        t<number> : number of frames to record
	s[b,B]  : camera size : b=boson320, B=boson640
	[0..9]  : linux video port

./BosonUSB   ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB r ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB y ->  opens Boson320 /dev/video0  in AGC-8bits mode
./BosonUSB sB 1    ->  opens Boson640 /dev/video1  in RAW16 mode
./BosonUSB sB y 2  ->  opens Boson640 /dev/video2  in AGC-8bits mode
./BosonUSB fcap -> creates a folder named 'cap' and inside TIFF files (raw16, agc, yuv) will be located.

*/

#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <opencv2/opencv.hpp>
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>


#define YUV   0
#define RAW16 1

using namespace cv;

#define v_major 1
#define v_minor 0

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

// Need to clean video for linux structs to avoid some random initializations problems (not always present)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

// Global variables to keep this simple
int width;
int height;

// Types of sensors supported
enum sensor_types {
  Boson320, Boson640
};

class Boson {
public:
	Boson(const std::string& video_, const int video_mode_);
    ~Boson();

    bool read_frame(cv::Mat& thermal_img);

    void AGC_Basic_Linear(cv::Mat& input_16, cv::Mat& output_8);
    void print_help();

private:
	std::string video;
	int video_mode;

    int ret;
	int fd;
	int i;
	struct v4l2_capability cap;
	// long frame=0;     // First frame number enumeration
	// char label[50];   // To display the information
	// char thermal_sensor_name[20];  // To store the sensor name
	// char filename[60];  // PATH/File_count
	// char folder_name[30];  // To store the folder name
    // char video_frames_str[30];
	// Default Program options
	int  video_frames=0;
	int  zoom_enable=0;
	int  record_enable=0;
	sensor_types my_thermal=Boson640;

    int height;
    int width;

    cv::Mat thermal_raw16;
    cv::Mat thermal_agc8;
	cv::Mat thermal_mono8;

    struct v4l2_format format;
    struct v4l2_requestbuffers bufrequest;
    struct v4l2_buffer bufferinfo;
    int type;
};