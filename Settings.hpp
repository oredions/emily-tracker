#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class Settings {
public:
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
    // The index of the camera have to be checked manually
    int video_capture_source = 0;

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

    //string video_capture_source = "rtsp://10.201.147.238:5000/screen";
    //string video_capture_source = "http://195.67.26.73/mjpg/video.mjpg";

    // HDMI stream from Teradek VidiU from Fotokite or 3DR Solo
    ////////////////////////////////////////////////////////////////////////////////

    //string video_capture_source = "rtmp://127.0.0.1/EMILY_Tracker/fotokite";

    ////////////////////////////////////////////////////////////////////////////////
    // Video files
    ////////////////////////////////////////////////////////////////////////////

    // Lake Bryan AI Robotic class field test 2016 03 31
    //string video_capture_source = "input/2016_03_31_lake_bryan.mp4";
    //int saturation_min = 52;
    //int value_min = 10;

    // Fort Bend floods 2016 04 26
    //string video_capture_source = "input/2016_04_26_fort_bend.mp4";
    //int saturation_min = 120;
    //int value_min = 100;

    // Lake Bryan AI Robotics class final 2016 05 10
    //string video_capture_source = "input/2016_05_10_lake_bryan.mov";
    //int saturation_min = 10;
    //int value_min = 10;

    // Lab 2016 07 05
    //string video_capture_source = "input/2016_07_05_lab.avi";
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

    ////////////////////////////////////////////////////////////////////////////////
    // Communication
    ////////////////////////////////////////////////////////////////////////////////

    // IP address and port of EMILY control computer to which to send throttle and
    // rudder valuers.

    const char * IP_ADDRESS = "192.168.1.4";
    const short PORT = 5007;

    ////////////////////////////////////////////////////////////////////////////////
    // GUI Parameters
    ////////////////////////////////////////////////////////////////////////////////

    // Enable four frame mode to get more detailed GUI.
    //
    // Two frame mode shows:
    // 1. threshold
    // 2. original image with tracking information
    //
    // Four frame mode shows:
    // 1. blurred image
    // 2. threshold
    // 3. eroded dilated threshold
    // 4. original image with tracking information
#define FOUR_FRAME_MODE

    // Main window name
    const string MAIN_WINDOW = "EMILY Tracker";

    // Histogram window name
    const string HISTOGRAM_WINDOW = "Histogram";

    // Object position crosshairs color
    const Scalar LOCATION_COLOR = Scalar(0, 255, 0);

    // Object position crosshairs thickness
    const int LOCATION_THICKNESS = 1;

    // Object pose line color
    const Scalar POSE_LINE_COLOR = Scalar(0, 255, 255);

    // Thickness of pose estimation lines
    const int POSE_LINE_THICKNESS = 1;

    // Thickness of heading estimation lines
    const int HEADING_LINE_THICKNESS = 1; // MOD change to 4 for lab testing

    // Length of the heading line
    const int HEADING_LINE_LENGTH = 20; // MOD change to 200 for lab testing

    // Target color
    const Scalar TARGET_COLOR = Scalar(0, 0, 255);

    // Target crosshairs radius
    const int TARGET_RADIUS = 10;

    ////////////////////////////////////////////////////////////////////////////////
    // Algorithm Static Parameters
    ////////////////////////////////////////////////////////////////////////////////

    // Input will be resized to this number of lines to speed up the processing
    //const int PROCESSING_VIDEO_HEIGHT_LIMIT = 640; // MOD webcam resolution
    // Higher resolution will be better if EMILY is in the distance
    const int PROCESSING_VIDEO_HEIGHT_LIMIT = 1200;

    // Blob size restrictions. Blobs outside of this range will be ignored.
    const int MIN_BLOB_AREA = 1 * 1;
    int MAX_BLOB_AREA;

};

#endif /* SETTINGS_HPP */