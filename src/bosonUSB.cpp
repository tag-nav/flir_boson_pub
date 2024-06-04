#include "bosonUSB.h"



Boson::Boson(const std::string& video_, const int video_mode_) {
	video = video_;
	video_mode = video_mode_;
	
	// Display Help
	// print_help();

	// Video device by default
	// sprintf(video, "/dev/video0");
	// sprintf(thermal_sensor_name, "Boson_640");

	// // Read command line arguments
	// for (i=0; i<argc; i++) {
	// 	// Check if RAW16 video is desired
	// 	if ( argv[i][0]=='r') {
	// 		video_mode=RAW16;
	// 	}
	// 	// Check if AGC video is desired
	// 	if ( argv[i][0]=='y') {
	// 		video_mode=YUV;
	// 	}
	// 	// Check if ZOOM to 640x512 is enabled
	// 	if ( argv[i][0]=='z') {
    //     		zoom_enable=1;
    //   		}
	// 	// Check if recording is enabled
	// 	if ( argv[i][0]=='f') {  // File name has to be more than two chars
    //         		record_enable=1;
    //         		if ( strlen(argv[i])>2 ) {
    //             		strcpy(folder_name, argv[i]+1);
    //         		}
    //   		}
	// 	// Look for type/size of sensor
	// 	if ( argv[i][0]=='s') {
    //       		switch ( argv[i][1] ) {
    //         			case 'B'/* value */:
    //             			my_thermal=Boson640;
    //            				sprintf(thermal_sensor_name, "Boson_640");
	// 	        	        break;
    //         			default:
	// 			        my_thermal=Boson320;
	// 			        sprintf(thermal_sensor_name, "Boson_320");
    //       		}
    //   		}
	// 	// Look for feedback in ASCII
	// 	if (argv[i][0]>='0' && argv[i][0]<='9') {
	// 		sprintf(video, "/dev/video%c",argv[i][0]);
	// 	}
	// 	// Look for frame count
    //     	if ( argv[i][0]=='t') {
    //         		if ( strlen(argv[i])>=2 ) {
	// 			strcpy(video_frames_str, argv[i]+1);
    //                             video_frames = atoi( video_frames_str );
    //                             printf(WHT ">>> Number of frames to record =" YEL "%i" WHT "\n", video_frames);
    //         		}	
    //     	}
  	// }

	// // Folder name
	// if (record_enable==1) {
	// 	if ( strlen(folder_name)<=1 ) {  // File name has to be more than two chars
	// 	        strcpy(folder_name, thermal_sensor_name);
	//         }
	//         mkdir(folder_name, 0700);
	//         chdir(folder_name);
    //             printf(WHT ">>> Folder " YEL "%s" WHT " selected to record files\n", folder_name);
  	// }

	// // Printf Sensor defined
	// printf(WHT ">>> " YEL "%s" WHT " selected\n", thermal_sensor_name);

	// We open the Video Device
	printf(WHT ">>> " YEL "%s" WHT " selected\n", video.c_str());
	if((fd = open(video.c_str(), O_RDWR)) < 0){
		perror(RED "Error : OPEN. Invalid Video Device" WHT "\n");
		exit(1);
	}

	// Check VideoCapture mode is available
	if(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0){
	    perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT "\n");
	    exit(1);
	}

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
		fprintf(stderr, RED "The device does not handle single-planar video capture." WHT "\n");
		exit(1);
	}

	// struct v4l2_format format;
	
	CLEAR(format);

	// Two different FORMAT modes, 8 bits vs RAW16
	if (video_mode==RAW16) {
		printf(WHT ">>> " YEL "16 bits " WHT "capture selected\n");

		// I am requiring thermal 16 bits mode
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

		// Select the frame SIZE (will depend on the type of sensor)
		switch (my_thermal) {
			case Boson320:  // Boson320
			          	width=320;
				        height=256;
				        break;
		        case Boson640:  // Boson640
				        width=640;
				        height=512;
				        break;
			default:  // Boson320
				        width=320;
				        height=256;
				        break;
		 }

	} else { // 8- bits is always 640x512 (even for a Boson 320)
		 printf(WHT ">>> " YEL "8 bits " WHT "YUV selected\n");
	         format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420; // thermal, works   LUMA, full Cr, full Cb
		 width = 640;
		 height = 512;
	}

	// Common varibles
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;

	// request desired FORMAT
	if(ioctl(fd, VIDIOC_S_FMT, &format) < 0){
		perror(RED "VIDIOC_S_FMT" WHT);
		exit(1);
	}

	// we need to inform the device about buffers to use.
	// and we need to allocate them.
	// we’ll use a single buffer, and map our memory using mmap.
	// All this information is sent using the VIDIOC_REQBUFS call and a
	// v4l2_requestbuffers structure:
	// struct v4l2_requestbuffers bufrequest;
	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count = 1;   // we are asking for one buffer

	if(ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0){
		perror(RED "VIDIOC_REQBUFS" WHT);
		exit(1);
	}

	// Now that the device knows how to provide its data,
	// we need to ask it about the amount of memory it needs,
	// and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
	// and its v4l2_buffer structure.

	// struct v4l2_buffer bufferinfo;
	memset(&bufferinfo, 0, sizeof(bufferinfo));

	bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufferinfo.memory = V4L2_MEMORY_MMAP;
	bufferinfo.index = 0;

	if(ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
		perror(RED "VIDIOC_QUERYBUF" WHT);
		exit(1);
	}


	// map fd+offset into a process location (kernel will decide due to our NULL). lenght and
	// properties are also passed
	printf(WHT ">>> Image width  =" YEL "%i" WHT "\n", width);
	printf(WHT ">>> Image height =" YEL "%i" WHT "\n", height);
	printf(WHT ">>> Buffer lenght=" YEL "%i" WHT "\n", bufferinfo.length);

	void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);

	if(buffer_start == MAP_FAILED){
		perror(RED "mmap" WHT);
		exit(1);
	}

	// Fill this buffer with ceros. Initialization. Optional but nice to do
	memset(buffer_start, 0, bufferinfo.length);

	// Activate streaming
	type = bufferinfo.type;
	if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
		perror(RED "VIDIOC_STREAMON" WHT);
		exit(1);
	}

	// Declarations for RAW16 representation
    // Will be used in case we are reading RAW16 format
	// Boson320 , Boson 640
	thermal_raw16 = cv::Mat(height, width, CV_16U, buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
	
	int luma_height;
	int luma_width;
	int color_space;

	// Declarations for 8bits YCbCr mode
        // Will be used in case we are reading YUV format
	// Boson320, 640 :  4:2:0
	luma_height = height+height/2;
	luma_width = width;
	color_space = CV_8UC1;
 	thermal_agc8 = cv::Mat(luma_height, luma_width,  color_space, buffer_start);  // OpenCV input buffer
	
	thermal_mono8 = cv::Mat(height, width, CV_8U, 1); // OpenCV output buffer : Data used to display the video
}


Boson::~Boson() {
    // // Deactivate streaming
	if( ioctl(fd, VIDIOC_STREAMOFF, &type) < 0 ){
		perror(RED "VIDIOC_STREAMOFF" WHT);
		exit(1);
	};

	close(fd);
}

bool Boson::read_frame(cv::Mat& thermal_img) {
	// Put the buffer in the incoming queue.
	if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
		perror(RED "VIDIOC_QBUF" WHT);
		exit(1);
	}

	// The buffer's waiting in the outgoing queue.
	if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
		perror(RED "VIDIOC_QBUF" WHT);
		exit(1);
	}


	// -----------------------------
	// RAW16 DATA
	if ( video_mode==RAW16 ) {
		thermal_img = thermal_raw16;
		
		// Below is just for visualization
		AGC_Basic_Linear(thermal_raw16, thermal_mono8);

		if (thermal_mono8.empty()) {
			perror(RED "thermal_mono8 empty" WHT);
		}

		cv::imshow("RAW16", thermal_mono8);
		cv::waitKey(1);
		
	}
	// ---------------------------------
	// DATA in YUV
	else {	// Video is in 8 bits YUV
		thermal_img = thermal_agc8;
		
		// Below is just for visualization
		cv::cvtColor(thermal_agc8, thermal_mono8, COLOR_YUV2GRAY_I420, 0 );   // 4:2:0 family instead of 4:2:2 ...

		if (thermal_mono8.empty()) {
			perror(RED "thermal_mono8 empty" WHT);
		}

		cv::imshow("AGC-8", thermal_mono8);
		cv::waitKey(1);		
	}

	return true;
}

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void Boson::AGC_Basic_Linear(cv::Mat& input_16, cv::Mat& output_8) {
	int i, j;  // aux variables

	// auxiliary variables for AGC calcultion
	unsigned int max1=0;         // 16 bits
	unsigned int min1=0xFFFF;    // 16 bits
	unsigned int value1, value2, value3, value4;

	// RUN a super basic AGC
	for (i=0; i<height; i++) {
		for (j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			if ( value3 <= min1 ) {
				min1 = value3;
			}
			if ( value3 >= max1 ) {
				max1 = value3;
			}
			//printf("%X.%X.%X  ", value1, value2, value3);
		}
	}
	//printf("max1=%04X, min1=%04X\n", max1, min1);

	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
			// printf("%04X \n", value4);

			output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);
		}
	}

}

// HELP INFORMATION
void Boson::print_help() {
	printf(CYN "Boson Capture and Record Video tool v%i.%i" WHT "\n", v_major, v_minor);
	printf(CYN "FLIR Systems" WHT "\n\n");
	printf(WHT "use : " YEL "'BosonUSB r' " WHT "to capture in raw-16 bits mode   (default)\n");
	printf(WHT "Use : " YEL "'BosonUSB y' " WHT "to capture in agc-8  bits mode\n");
  	printf(WHT "Use : " YEL "'BosonUSB z' " WHT "Zoom to 640x512 (only in RAW) mode  (default ZOOM OFF)\n");
	printf(WHT "Use : " YEL "'BosonUSB f<name>' " WHT "record TIFFS in Folder <NAME>\n");
	printf(WHT "Use : " YEL "'BosonUSB f<name> t<frame_count>' " WHT "record TIFFS in Folder <NAME> and stop recording after <FRAME_COUNT> frames\n");
	printf(WHT "Use : " YEL "'BosonUSB [0..9]'   " WHT "to open /dev/Video[0..9]  (default 0)\n");
	printf(WHT "Use : " YEL "'BosonUSB s[b,B]'   " WHT "b=boson320, B=boson640   (default 320)\n");
	printf(WHT "Press " YEL "'q' in video window " WHT " to quit\n");
	printf("\n");
}

// /* ---------------------------- Main Function ---------------------------------------*/
// // ENTRY POINT
// int main(int argc, char** argv )
// {
//     Boson boson;

// 	// Reaad frame, do AGC, paint frame
// 	for (;;) {
// 		boson.read();
// 	}
// 	// Finish Loop . Exiting.
// }
