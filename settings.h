#include "opencv2/opencv.hpp"

using namespace cv;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Input
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Video streams
////////////////////////////////////////////////////////////////////////////

// Logitech USB Camera
////////////////////////////////////////////////////////////////////////////////

// Camera index (sometimes the external USB camera is on 0 and sometimes on 1)
VideoCapture video_capture(1);

// Saturation and value limits
int saturation_min = 120;
int value_min = 10;

// Calibration Data
Mat camera_intrinsic_matrix = (Mat_<double>(3, 3) <<
        1.4006590516106746e+03, 0., 7.9950000000000000e+02,
        0., 1.4006590516106746e+03, 5.9950000000000000e+02,
        0., 0., 1.);

Mat camera_distortion_vector = (Mat_<double>(1, 5) <<
        5.4296267484343998e-02, -5.9413030935709088e-01, 0., 0., 1.9565543630464968e+00);

// Inogeni
////////////////////////////////////////////////////////////////////////////////

// If USB device is Inogeni, uncomment this to set correct FPS
//#define INOGENI

// Screen mirroring application from DJI tablet
////////////////////////////////////////////////////////////////////////////////

//VideoCapture video_capture("rtsp://10.201.147.238:5000/screen");

// HDMI stream from Teradek VidiU from Fotokite or 3DR Solo
////////////////////////////////////////////////////////////////////////////////

//VideoCapture video_capture("rtmp://127.0.0.1/EMILY_Tracker/fotokite");

////////////////////////////////////////////////////////////////////////////////
// Video files
////////////////////////////////////////////////////////////////////////////

// Lake Bryan AI Robotic class field test 2016 03 31
//VideoCapture video_capture("input/2016_03_31_lake_bryan.mp4");
//int saturation_min = 52;
//int value_min = 10;

// Fort Bend floods 2016 04 26
//VideoCapture video_capture("input/2016_04_26_fort_bend.mp4");
//int saturation_min = 120;
//int value_min = 100;

// Lake Bryan AI Robotics class final 2016 05 10
//VideoCapture video_capture("input/2016_05_10_lake_bryan.mov");
//int saturation_min = 10;
//int value_min = 10;

// Lab 2016 07 05
//VideoCapture video_capture("input/2016_07_05_lab.avi");
//int saturation_min = 130;
//int value_min = 10;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Algorithm Variable parameters
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Hue 1 range for thresholding
int hue_1_min = 0;
int hue_1_max = 10;

// Hue 2 range for thresholding
int hue_2_min = 160;
//int hue_2_min = 180; // Works well for lab
int hue_2_max = 180;

// Saturation range for thresholding
int saturation_max = 255;

// Value range for thresholding
int value_max = 255;

// Gaussian blur kernel size
int blur_kernel_size = 21;

// When the centroid of EMILY gets closer or equal to this number of pixels, it will consider target to be reached
int target_radius = 30;

// Slowing distance as multiple of target radius. When this threshold is reached, EMILY will start linearly slowing down.
int slowing_distance = 3;

// Value of the proportional parameter of PID
int proportional = 20;

// Camera angle in degrees
int camera_angle_degrees = 45;
double camera_angle_radians;

// Erode size
int erode_size = 2;

// Dilate size
int dilate_size = 16;

// EMILY location history size to estimate heading
const int EMILY_LOCATION_HISTORY_SIZE = 50;