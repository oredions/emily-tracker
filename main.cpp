/** 
 * @file    main.cpp
 * @author  Jan Dufek
 * @date    07/01/2016
 * @version 2.0
 *  
 * This program is used to track EMILY unmanned surface vehicle in video feed
 * and to get its coordinates and pose.
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "control.h"

#include <sys/socket.h>
#include <netdb.h>

#include <stdlib.h>
#include <string.h>

#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

////////////////////////////////////////////////////////////////////////////////
// TODOs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Name spaces
////////////////////////////////////////////////////////////////////////////////

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Input
////////////////////////////////////////////////////////////////////////////////

// Turn on video stream. If defined, it will take input from stream. If not, it
// will take input from video file.
//#define VIDEO_STREAM

#ifdef VIDEO_STREAM


// Video streams
////////////////////////////////////////////////////////////////////////////

// Screen mirroring application from DJI tablet
//VideoCapture video_capture("rtsp://10.201.147.238:5000/screen");

// HDMI stream from Teradek VidiU from Fotokite or 3DR Solo
VideoCapture video_capture("rtmp://127.0.0.1/EMILY_Tracker/fotokite");

#else

// Video files
////////////////////////////////////////////////////////////////////////////

// Lake Bryan AI Robotic class field test 2016 03 31
//VideoCapture video_capture("input/2016_03_31_lake_bryan.mp4");

// Fort Bend floods 2016 04 26
//VideoCapture video_capture("input/2016_04_26_fort_bend.mp4");

// Lake Bryan AI Robotics class final 2016 05 10
//VideoCapture video_capture("input/2016_05_10_lake_bryan.mov");

// Lab 2016 07 05
//VideoCapture video_capture("input/2016_07_05_lab.avi");

// USB web camera
// Sometimes the external USB camera is on 0 and sometimes on 1
VideoCapture video_capture(0);

#endif

////////////////////////////////////////////////////////////////////////////////
// Algorithm
////////////////////////////////////////////////////////////////////////////////

// Enable advanced Camshift algorithm. It will let you choose object
// object of interest, computes histogram, and tracks it using mean shift.
#define CAMSHIFT

////////////////////////////////////////////////////////////////////////////////
// Analysis
////////////////////////////////////////////////////////////////////////////////

// Enables analysis of localization performance. As soon as an object is
// selected, it will log pixel distance from the mouse cursor. The operator
// have to keep the cursor in the centroid of EMILY all the time to mark its
// true position.
//#define ANALYSIS;

////////////////////////////////////////////////////////////////////////////////
// Communication
////////////////////////////////////////////////////////////////////////////////

// IP address and port of EMILY control computer to which to send throttle and
// rudder valuers.

const char * IP_ADDRESS = "192.168.1.4";
const short PORT = 5005;

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

////////////////////////////////////////////////////////////////////////////////
// Algorithm Variable parameters
////////////////////////////////////////////////////////////////////////////////

// Hue 1 range for thresholding
int hue_1_min = 0;
int hue_1_max = 10;

// Hue 2 range for thresholding
int hue_2_min = 160;
//int hue_2_min = 180; // Works well for lab

int hue_2_max = 180;

// Saturation range for thresholding
//int saturation_min = 52; // Works well for Lake Bryan 1
//int saturation_min = 120; // Works well for Fort Bend
int saturation_min = 10; // Works well for Lake Bryan 2
//int saturation_min = 130; // Works well for lab

int saturation_max = 255;

// Value range for thresholding
//int value_min = 10; // Works well for Lake Bryan 1
//int value_min = 100; // Works well for Fort Bend
int value_min = 10; // Works well for Lake Bryan 2

int value_max = 255;

// Gaussian blur kernel size
int blur_kernel_size = 21;

// When the centroid of EMILY gets closer or equal to this number of pixels, it will consider target to be reached
int target_radius = 10;

// Value of the proportional parameter of PID
int proportional = 10;

// Erode size
int erode_size = 2;

// Dilate size
int dilate_size = 16;

// EMILY location history size to estimate heading
const int EMILY_LOCATION_HISTORY_SIZE = 50;

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
// Global variables
////////////////////////////////////////////////////////////////////////////////

// Back projection mode toggle
bool back_projection_mode = false;

// Select object semaphore
bool select_object = false;

// Track object mode toggle
int object_selected = 0;

// Original point of click
Point origin;

// Object selection
Rect selection;

// New resized size of video used in processing
Size resized_video_size;

// Indicates that resizing is necessary
bool resize_video = false;

// Original frame
Mat original_frame;

// Target location for EMILY to go to
Point target_location;

// EMILY location
Point emily_location;

// EMILY location history
Point emily_location_history[EMILY_LOCATION_HISTORY_SIZE];

#ifdef ANALYSIS

Point mouse_location;

#endif

// EMILY motion angle
double emily_angle;

// Target was reached
bool target_reached = false;

// Timer to estimate EMILY heading
int emily_location_history_pointer;

// EMILY pose
Point emily_pose_point_1;
Point emily_pose_point_2;

/**
 * Convert integer to string.
 * 
 * @param number integer to be converted to string
 * 
 */
string int_to_string(int number) {
    stringstream stringStream;
    stringStream << number;
    return stringStream.str();
}

/**
 * Trackbar handler. Called when track bar is clicked on.
 * 
 */
void on_trackbar(int, void*) {

    // Gaussian kernel size must be positive and odd. Or, it can be zero’s and
    // then it is computed from sigma.
    if (blur_kernel_size % 2 == 0) {
        blur_kernel_size++;
    }

    // Erode size cannot be 0
    if (erode_size == 0) {
        erode_size++;
    }

    // Dilate size cannot be 0
    if (dilate_size == 0) {
        dilate_size++;
    }
}

/**
 * Shows the main window and creates track bars.
 * 
 */
void create_main_window() {

    // Show new window
    namedWindow(MAIN_WINDOW, CV_GUI_NORMAL);

#ifndef CAMSHIFT

    // Hue 1 trackbars
    createTrackbar("H 1 Min", MAIN_WINDOW, &hue_1_min, 180, on_trackbar);
    createTrackbar("H 1 Max", MAIN_WINDOW, &hue_1_max, 180, on_trackbar);

    // Hue 2 trackbars
    createTrackbar("H 2 Min", MAIN_WINDOW, &hue_2_min, 180, on_trackbar);
    createTrackbar("H 2 Max", MAIN_WINDOW, &hue_2_max, 180, on_trackbar);

#endif

    // Saturation trackbars
    createTrackbar("S Min", MAIN_WINDOW, &saturation_min, 255, on_trackbar);
    createTrackbar("S Max", MAIN_WINDOW, &saturation_max, 255, on_trackbar);

    // Value trackbars
    createTrackbar("V Min", MAIN_WINDOW, &value_min, 255, on_trackbar);
    createTrackbar("V Max", MAIN_WINDOW, &value_max, 255, on_trackbar);

    // Gaussian blur trackbar
    createTrackbar("Blur", MAIN_WINDOW, &blur_kernel_size, min(resized_video_size.height, resized_video_size.width), on_trackbar);

    // Target reached radius trackbar
    createTrackbar("Radius", MAIN_WINDOW, &target_radius, min(resized_video_size.height, resized_video_size.width), on_trackbar);

    // Proportional controller trackbar
    createTrackbar("Proportion", MAIN_WINDOW, &proportional, 100, on_trackbar);

#ifndef CAMSHIFT    

    // Erode size trackbar
    createTrackbar("Erode", MAIN_WINDOW, &erode_size, min(resized_video_size.height, resized_video_size.width), on_trackbar);

    // Dilate size trackbar
    createTrackbar("Dilate", MAIN_WINDOW, &dilate_size, min(resized_video_size.height, resized_video_size.width), on_trackbar);

#endif    

}

/**
 * Select object of interest in image.
 * 
 */
static void onMouse(int event, int x, int y, int flags, void*) {

    // Select object mode
    if (select_object) {

        // Get selected rectangle
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = abs(x - origin.x);
        selection.height = abs(y - origin.y);

        // Get intersection with the original image
        selection &= Rect(0, 0, original_frame.cols, original_frame.rows);
    }

    // Check mouse button
    switch (event) {

            // Right drag is reserved for moving over the image
            // Left click context menu is disabled
            // Scrool is reserved for zoom

            // Right drag to select EMILY
        case EVENT_RBUTTONDOWN:

            // Save current point as click origin
            origin = Point(x, y);

            // Initialize rectangle
            selection = Rect(x, y, 0, 0);

            // Start selection
            select_object = true;

            break;

        case EVENT_RBUTTONUP:

            // End selection
            select_object = false;

            // If the selection has been made, start tracking
            if (selection.width > 0 && selection.height > 0) {
                object_selected = -1;
            }

            break;

            // Left double click to choose target
        case EVENT_LBUTTONDBLCLK:

            // Get location of the target
            target_location = Point(x, y);
            target_reached = false;

            //cout << int_to_string(target_location.x) + " " + int_to_string(target_location.y) << endl;

            break;
            
        #ifdef ANALYSIS

        case EVENT_MOUSEMOVE:
            
            // Get mouse location
            mouse_location = Point(x, y);
            
            // Print mouse location
            //cout << int_to_string(mouse_location.x) + " " + int_to_string(mouse_location.y) << endl;
            
            break;
            
        #endif
    }
}

/**
 * Draws position of the object as crosshairs with the center in the object's
 * centroid.
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param radius radius of crosshairs
 * @param frame frame to which draw into
 */
void draw_object_position(int x, int y, double radius, Mat &frame) {

#ifndef CAMSHIFT

    // Circle
    circle(frame, Point(x, y), radius, LOCATION_COLOR, LOCATION_THICKNESS);

#endif

    // Lines
    if (y - radius > 0) {
        line(frame, Point(x, y), Point(x, y - radius), LOCATION_COLOR, LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(x, 0), LOCATION_COLOR, LOCATION_THICKNESS);
    }

    if (y + radius < resized_video_size.height) {
        line(frame, Point(x, y), Point(x, y + radius), LOCATION_COLOR, LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(x, resized_video_size.height), LOCATION_COLOR, LOCATION_THICKNESS);
    }

    if (x - radius > 0) {
        line(frame, Point(x, y), Point(x - radius, y), LOCATION_COLOR, LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(0, y), LOCATION_COLOR, LOCATION_THICKNESS);
    }

    if (x + radius < resized_video_size.width) {
        line(frame, Point(x, y), Point(x + radius, y), LOCATION_COLOR, LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(resized_video_size.width, y), LOCATION_COLOR, LOCATION_THICKNESS);
    }

    // Text coordinates
    putText(frame, "[" + int_to_string(x) + "," + int_to_string(y) + "]", Point(x, y + radius + 20), 1, 1, LOCATION_COLOR, 1, 8);
}

/**
 * Draws axis of given rotated rectangle. Axis is an principal axis of symmetry.
 * 
 * @param rectangle rotated rectangle for which to draw principal axis
 */
void draw_principal_axis(RotatedRect rectangle) {

    // Get EMILY pose as the principal symmetry axis of bounding rectangle

    // Get points of bounding rectangle
    Point2f rectangle_points[4];
    rectangle.points(rectangle_points);

    // Initialize variables to look for the rectangle shortest side
    double shortest_axis = DBL_MAX;
    int shortest_axis_index;

    // For each side
    for (int j = 0; j < 4; j++) {

        // Line length
        double line_length = norm(rectangle_points[j] - rectangle_points[(j + 1) % 4]);

        if (line_length < shortest_axis) {
            shortest_axis = line_length;
            shortest_axis_index = j;
        }

        // Draw line of the bounding rectangle
        //line( frame, rectangle_points[j], rectangle_points[(j + 1) % 4], color, 1, 8 );
    }

    // Get midpoints of the shortest sides
    Point shortest_axis_midpoint_1 = (rectangle_points[shortest_axis_index] + rectangle_points[(shortest_axis_index + 1) % 4]) * 0.5;
    Point shortest_axis_midpoint_2 = (rectangle_points[(shortest_axis_index + 2) % 4] + rectangle_points[(shortest_axis_index + 3) % 4]) * 0.5;

    // Save EMILY pose
    emily_pose_point_1 = shortest_axis_midpoint_1;
    emily_pose_point_2 = shortest_axis_midpoint_2;

    // Draw line representing principal axis of symmetry
    line(original_frame, shortest_axis_midpoint_1, shortest_axis_midpoint_2, POSE_LINE_COLOR, POSE_LINE_THICKNESS, 8);

}

double get_size(RotatedRect rectangle) {

    // Get points of bounding rectangle
    Point2f rectangle_points[4];
    rectangle.points(rectangle_points);

    // Initialize variables to look for the rectangle shortest side
    double shortest_axis = DBL_MAX;
    int shortest_axis_index;

    // For each side
    for (int j = 0; j < 4; j++) {

        // Line length
        double line_length = norm(rectangle_points[j] - rectangle_points[(j + 1) % 4]);

        if (line_length < shortest_axis) {
            shortest_axis = line_length;
            shortest_axis_index = j;
        }
    }

    // Get midpoints of the shortest sides
    Point shortest_axis_midpoint_1 = (rectangle_points[shortest_axis_index] + rectangle_points[(shortest_axis_index + 1) % 4]) * 0.5;
    Point shortest_axis_midpoint_2 = (rectangle_points[(shortest_axis_index + 2) % 4] + rectangle_points[(shortest_axis_index + 3) % 4]) * 0.5;

    // Return size
    return sqrt(pow(shortest_axis_midpoint_1.x - shortest_axis_midpoint_2.x, 2) + pow(shortest_axis_midpoint_1.y - shortest_axis_midpoint_2.y, 2)) / 2;

}

/**
 * Unused.
 * 
 * @param img
 * @param p
 * @param q
 * @param colour
 * @param scale
 */
void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2) {
    double angle;
    double hypotenuse;
    angle = atan2((double) p.y - q.y, (double) p.x - q.x); // angle in radians
    hypotenuse = sqrt((double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    //    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
    //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    line(img, p, q, colour, 1, CV_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, CV_AA);
}

/**
 * Unused.
 * 
 * @param pts
 * @param img
 * @return 
 */
double getOrientation(const vector<Point> &pts, Mat &img) {
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int> (pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i) {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int> (pca_analysis.mean.at<double>(0, 0)),
            static_cast<int> (pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i) {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int> (eigen_vecs[0].x * eigen_val[0]), static_cast<int> (eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int> (eigen_vecs[1].x * eigen_val[1]), static_cast<int> (eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle;
}

/**
 * Track EMILY in video feed.
 * 
 */
int main(int argc, char** argv) {

    ////////////////////////////////////////////////////////////////////////////
    // Output video initialization
    //////////////////////////////////////////////////////////////////////////// 

    // Get FPS of the input video
    double input_video_fps = video_capture.get(CV_CAP_PROP_FPS);

    // If the input is video stream, we have to calculate FPS manually
    if (input_video_fps == 0) {

        // Number of sample frames to capture
        int num_sample_frames = 200;

        // Start and end times
        time_t start, end;

        // Sample video frame
        Mat sample_frame;

        // Start timer
        time(&start);

        // Load sample frames
        for (int i = 0; i < num_sample_frames; i++) {
            video_capture >> sample_frame;
        }

        // End timer
        time(&end);

        // Compute elapsed time
        double time_difference = difftime(end, start);

        // Calculate frames per second
        input_video_fps = num_sample_frames / time_difference;
        
        // The maximum frame rate from MPEG 4 is 65.535
        if (input_video_fps > 65.535) {
            input_video_fps = 65.535;
        }
        
    }

    // Get the size of input video
    Size input_video_size(video_capture.get(CV_CAP_PROP_FRAME_WIDTH), video_capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    // If the input video exceeds processing video size limits, we will have to resize it
    if (input_video_size.height > PROCESSING_VIDEO_HEIGHT_LIMIT) {

        // Compute scale ratio
        double ratio = (double) PROCESSING_VIDEO_HEIGHT_LIMIT / input_video_size.height;

        // Compute new width
        int new_video_width = input_video_size.width * ratio;

        // Set new size
        resized_video_size.height = PROCESSING_VIDEO_HEIGHT_LIMIT;
        resized_video_size.width = new_video_width;

        // Set parameter for maximum blob area
        MAX_BLOB_AREA = resized_video_size.height * resized_video_size.width;

        // Indicate that resizing is necessary
        resize_video = true;

    } else {

        // Set video size to original size
        resized_video_size.height = input_video_size.height;
        resized_video_size.width = input_video_size.width;

        // Set parameter for maximum blob area
        MAX_BLOB_AREA = resized_video_size.height * resized_video_size.width;

        // Indicate that resizing is not necessary
        resize_video = false;

    }

    // Codec used to output the video
    // This is higher size: int outputVideoCodec = CV_FOURCC('W','R','L','E');
    // This works navite on Mac: int outputVideoCodec = CV_FOURCC('m', 'p', '4', 'v');
    // This works with ffmpeg
    int output_video_codec = CV_FOURCC('D', 'I', 'V', 'X');

    // Output video name. It is in format year_month_day_hour_minute_second.avi.
    time_t raw_time;
    time(&raw_time);
    struct tm * local_time;
    local_time = localtime(&raw_time);
    char output_file_name[40];
    strftime(output_file_name, 40, "output/%Y_%m_%d_%H_%M_%S", local_time);
    string output_file_name_string(output_file_name);

    // Open output video file
    VideoWriter output_video(output_file_name_string + ".avi", output_video_codec, input_video_fps, resized_video_size, true);

    // Check if output video file was successfully opened
    if (!output_video.isOpened()) {
        cout << "Cannot open the output video file " << output_file_name_string + ".avi" << " for write." << endl;
        return -1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Log
    ////////////////////////////////////////////////////////////////////////////

    // Open log file
    ofstream log_file;
    log_file.open(output_file_name_string + ".txt");

    // Open rudder log file
    ofstream rudder_log_file;
    rudder_log_file.open(output_file_name_string + "_rudder.txt");

    // Open control log file
    ofstream throttle_log_file;
    throttle_log_file.open(output_file_name_string + "_throttle.txt");
    
    #ifdef ANALYSIS

    // Open error log file
    ofstream error_log_file;
    error_log_file.open(output_file_name_string + "_error.txt");
    
    #endif
    
    ////////////////////////////////////////////////////////////////////////////
    // GUI
    ////////////////////////////////////////////////////////////////////////////

    // Show main window including slide bars
    create_main_window();

#ifdef CAMSHIFT

    // Histogram window
    namedWindow("Histogram", 0);

#endif
    
    // Set mouse handler on main window to choose object of interest
    setMouseCallback(MAIN_WINDOW, onMouse, 0);

    ////////////////////////////////////////////////////////////////////////////
    // Local variables
    ////////////////////////////////////////////////////////////////////////////

    // Frame with edits for blob detection
    Mat blured_frame;

    // Rectangle representing object of interest
    Rect object_of_interest;

    // Size of histogram of object of interest
    int histogram_size = 16;

    // Histogram ranges
    float histogram_ranges[] = {0, 180};
    const float * pointer_histogram_ranges = histogram_ranges;

    // HSV hue
    Mat hue;

    // Threshold on saturation and value only. Hue is not thresholded.
    Mat saturation_value_threshold;

    // Histogram of object of interest
    Mat histogram;

    // Visualization of histogram
    Mat histogram_image = Mat::zeros(200, 320, CV_8UC3);

    // Back projection of histogram
    Mat back_projection;

    // Paused mode
    bool paused = false;

    ////////////////////////////////////////////////////////////////////////////
    // Initialization of communication
    ////////////////////////////////////////////////////////////////////////////

    // Create socket descriptor. We want to use datagram UDP.
    int socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_descriptor < 0) {
        cout << "Error creating socket descriptor." << endl;
    }

    // Create socket address
    struct sockaddr_in socket_address;
    socket_address.sin_family = AF_INET;
    socket_address.sin_addr.s_addr = inet_addr(IP_ADDRESS);
    socket_address.sin_port = htons(PORT);

    ////////////////////////////////////////////////////////////////////////////
    // Tracking
    ////////////////////////////////////////////////////////////////////////////

    // Iterate over each frame from the video input and wait between iterations.
    while (waitKey(1) != 27) {

        // If not paused       
        if (!paused) {

            // Read one frame
            video_capture >> original_frame;

            if (original_frame.empty()) {
                break;
            }

        }

        ////////////////////////////////////////////////////////////////////////
        // Global flags
        ////////////////////////////////////////////////////////////////////////

        bool target_set = target_location.x != 0 && target_location.y != 0;

        ////////////////////////////////////////////////////////////////////////
        // Thresholding
        ////////////////////////////////////////////////////////////////////////

        if (resize_video) {

            // Resize the input
            resize(original_frame, original_frame, resized_video_size, 0, 0, INTER_LANCZOS4);

        }

        // Blur
        GaussianBlur(original_frame, blured_frame, Size(blur_kernel_size, blur_kernel_size), 0, 0);

        // Convert to HSV
        Mat HSV_frame;
        cvtColor(blured_frame, HSV_frame, COLOR_BGR2HSV);

        // Equalize on value (V)
        vector<Mat> HSV_planes;
        split(HSV_frame, HSV_planes);
        equalizeHist(HSV_planes[2], HSV_planes[2]);
        merge(HSV_planes, HSV_frame);

#ifndef CAMSHIFT        

        // Threshold on lower red
        Mat lower_red_threshold;
        inRange(HSV_frame, cv::Scalar(hue_1_min, saturation_min, value_min), cv::Scalar(hue_1_max, saturation_max, value_max), lower_red_threshold);

        // Threshold on upper red
        Mat upper_red_threshold;
        inRange(HSV_frame, cv::Scalar(hue_2_min, saturation_min, value_min), cv::Scalar(hue_2_max, saturation_max, value_max), upper_red_threshold);

        // Add thresholds together
        Mat threshold;
        addWeighted(lower_red_threshold, 1.0, upper_red_threshold, 1.0, 0.0, threshold);

        // Erode to filter noise
        Mat eroded_dilated_threshold;
        Mat erode_element = getStructuringElement(MORPH_RECT, Size(erode_size, erode_size));
        erode(threshold, eroded_dilated_threshold, erode_element);
        erode(eroded_dilated_threshold, eroded_dilated_threshold, erode_element);

        // Dilate to make blobs more distinctive
        Mat dilate_element = getStructuringElement(MORPH_RECT, Size(dilate_size, dilate_size));
        dilate(eroded_dilated_threshold, eroded_dilated_threshold, dilate_element);
        dilate(eroded_dilated_threshold, eroded_dilated_threshold, dilate_element);

        ////////////////////////////////////////////////////////////////////////
        // Simple Blob Detector
        //
        // Does not work as good as object tracking
        ////////////////////////////////////////////////////////////////////////

        // Setup parameters
        SimpleBlobDetector::Params params;

        // Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 200;

        // Filter by color (only lightness)
        params.filterByColor = false;
        params.blobColor = 255;

        // Filter by area
        params.filterByArea = true;
        params.minArea = 50;

        // Filter by circularity
        params.filterByCircularity = false;
        params.minCircularity = 0.1;

        // Filter by convexity
        params.filterByConvexity = false;
        params.minConvexity = 0.85;

        // Filter by inertia
        params.filterByInertia = false;
        params.minInertiaRatio = 0.01;

        // Storage for blobs
        vector<KeyPoint> keypoints;

        // For OpenCV 2
#if CV_MAJOR_VERSION < 3

        // Set up detector with params
        SimpleBlobDetector detector(params);

        // Detect blobs
        detector.detect(eroded_dilated_threshold, keypoints);

        // For OpenCV 3
#else

        // Set up detector with params
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

        // Detect blobs
        detector->detect(eroded_dilated_threshold, keypoints);

#endif

        ////////////////////////////////////////////////////////////////////////
        // Object tracking
        ////////////////////////////////////////////////////////////////////////

        // x coordinate of the tracked object
        int x;

        // y coordinate of the tracked object
        int y;

        // Contours
        vector< vector<Point> > contours;

        // Hierarchy
        vector<Vec4i> hierarchy;

        // Find countours
        Mat contours_frame;
        eroded_dilated_threshold.copyTo(contours_frame);
        findContours(contours_frame, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

        // Was any object found?
        bool object_found = false;

        // If there is any hierarchy
        if (hierarchy.size() > 0) {

            // If there is multiple of the objects, take the largest one
            double max_area = 0;
            double max_area_object_x = 0;
            double max_area_object_y = 0;
            double max_area_contour_index = -1;

            // For each object
            for (int i = 0; i >= 0; i = hierarchy[i][0]) {

                // Get moments
                Moments moment = moments((Mat) contours[i]);

                // Get area
                double area = moment.m00;

                // Check if the area is within the set limit
                if (area > MIN_BLOB_AREA && area < MAX_BLOB_AREA) {

                    // Get x coordinate
                    x = moment.m10 / area;

                    // Get y coordinate
                    y = moment.m01 / area;

                    // Object of appropriate size was found
                    object_found = true;

                    // Check if the new biggest object was found
                    if (area > max_area) {
                        max_area = area;
                        max_area_object_x = x;
                        max_area_object_y = y;
                        max_area_contour_index = i;
                    }
                }

            }

            // We have found an object
            if (object_found == true) {

                ////////////////////////////////////////////////////////////
                // Estimate pose
                ////////////////////////////////////////////////////////////

                // Line fitting method
                //
                // Same as second principal axis from PCA. Does not work
                // well.
                ////////////////////////////////////////////////////////////

                //Vec4f lines;
                //fitLine(contours[max_area_contour_i], lines, CV_DIST_L2, 0, 0.01, 0.01);
                //int left_y = (-lines[2] * lines[1] / lines[0]) + lines[3];
                //int right_y = ((frame.cols - lines[2]) * lines[1] / lines[0]) + lines[3];
                //line(frame, Point(frame.cols-1, right_y), Point(0, left_y), Scalar(255,0,0), 2);

                // PCA method
                //
                // Does not work well.
                ////////////////////////////////////////////////////////////

                //getOrientation(contours[max_area_contour_i], frame);

                // EMILY pose using minimum ellipse
                //
                // Works better. It finds minimum bounding ellipse
                // of the contour. Then it gets bounding rectangle of that
                // ellipse. It determines the shortest sides and gets
                // midpoints of those shortest sides. The line connecting
                // those midpoints is the principal axis of EMILY.
                ////////////////////////////////////////////////////////////

                if (contours[max_area_contour_index].size() > 4) {

                    // Initialize minimum rectangle
                    RotatedRect min_rectangle;

                    // Initialize minimum ellipse
                    RotatedRect min_ellipse;

                    // Get minimum rectangle
                    min_rectangle = minAreaRect(Mat(contours[max_area_contour_index]));

                    // Get minimum ellipse
                    min_ellipse = fitEllipse(Mat(contours[max_area_contour_index]));

                    // Draw ellipse
                    //ellipse(original_frame, min_ellipse, Scalar(255,0,0), 2, 8 );

                    // Draw pose
                    draw_principal_axis(min_ellipse);

                    ////////////////////////////////////////////////////////////
                    // Draw EMILY location in the image
                    ////////////////////////////////////////////////////////////

                    // Compute object size
                    double object_size = get_size(min_ellipse);

                    // Draw object
                    draw_object_position(max_area_object_x, max_area_object_y, object_size, original_frame);
                    
                    // Save EMILY location
                    emily_location = Point(max_area_object_x, max_area_object_y);
                }
            }
        } else {

            // EMILY was not found in the image
            putText(original_frame, "EMILY not found!", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

        }

#else       

        if (!paused) {

            if (object_selected) {

                // Threshold on saturation and value, but not on hue
                inRange(HSV_frame, Scalar(0, saturation_min, value_min), Scalar(180, saturation_max, value_max), saturation_value_threshold);

                // Mix channels
                int chanels[] = {0, 0};
                hue.create(HSV_frame.size(), HSV_frame.depth());
                mixChannels(&HSV_frame, 1, &hue, 1, chanels, 1);

                // Object does not have histogram yet, so create it
                if (object_selected < 0) {

                    // Region of interest
                    Mat region_of_interest(hue, selection);

                    // Region of interest mask
                    Mat region_of_interest_mask(saturation_value_threshold, selection);

                    // Calculate histogram of region of interest
                    calcHist(&region_of_interest, 1, 0, region_of_interest_mask, histogram, 1, &histogram_size, &pointer_histogram_ranges);

                    // Normalize histogram
                    normalize(histogram, histogram, 0, 255, NORM_MINMAX);

                    // Set object of interest to selection
                    object_of_interest = selection;

                    // Begin tracking object
                    object_selected = 1;

                    // Create histogram visualization
                    histogram_image = Scalar::all(0);
                    int bins_width = histogram_image.cols / histogram_size;
                    Mat buffer(1, histogram_size, CV_8UC3);
                    for (int i = 0; i < histogram_size; i++)
                        buffer.at<Vec3b>(i) = Vec3b(saturate_cast<uchar> (i * 180. / histogram_size), 255, 255);
                    cvtColor(buffer, buffer, COLOR_HSV2BGR);
                    for (int i = 0; i < histogram_size; i++) {
                        int val = saturate_cast<int> (histogram.at<float> (i) * histogram_image.rows / 255);
                        rectangle(histogram_image, Point(i*bins_width, histogram_image.rows), Point((i + 1) * bins_width, histogram_image.rows - val), Scalar(buffer.at<Vec3b>(i)), -1, 8);
                    }
                }

                // Calculate back projection
                calcBackProject(&hue, 1, 0, histogram, back_projection, &pointer_histogram_ranges);

                // Apply back projection on saturation value threshold
                back_projection &= saturation_value_threshold;

                // CamShift algorithm
                RotatedRect tracking_box = CamShift(back_projection, object_of_interest, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));

                // Object of interest are is too small, so inflate the tracking box
                if (object_of_interest.area() <= 1) {
                    int cols = back_projection.cols;
                    int rows = back_projection.rows;
                    int new_rectangle_size = (MIN(cols, rows) + 5) / 6;
                    object_of_interest = Rect(object_of_interest.x - new_rectangle_size, object_of_interest.y - new_rectangle_size, object_of_interest.x + new_rectangle_size, object_of_interest.y + new_rectangle_size) & Rect(0, 0, cols, rows);
                }

                // We are in back projection mode
                if (back_projection_mode) {
                    cvtColor(back_projection, original_frame, COLOR_GRAY2BGR);
                }

                // Draw bounding ellipse
                if (tracking_box.size.height > 0 && tracking_box.size.width > 0) {
                    ellipse(original_frame, tracking_box, LOCATION_COLOR, LOCATION_THICKNESS, LINE_AA);

                    // Draw cross hairs
                    draw_object_position(tracking_box.center.x, tracking_box.center.y, min(tracking_box.size.width, tracking_box.size.height) / 2, original_frame);

                    // Draw pose
                    draw_principal_axis(tracking_box);

                    // Save EMILY location
                    emily_location = Point(tracking_box.center.x, tracking_box.center.y);

                }

            }
        } else if (object_selected < 0) {

            // Un pause if the selection has been made
            paused = false;
        }

        // Show the selection
        if (select_object && selection.width > 0 && selection.height > 0) {
            Mat roi(original_frame, selection);
            bitwise_not(roi, roi);
        }

        // Show the histogram
        imshow("Histogram", histogram_image);

        char character = (char) waitKey(10);
        if (character == 27)
            break;
        switch (character) {
            case 'b':

                // Toggle back projection mode
                back_projection_mode = !back_projection_mode;

                break;
            case 'c':

                // Stop tracking
                object_selected = 0;
                histogram_image = Scalar::all(0);

                break;
            case 'p':

                // Toggle pause
                paused = !paused;

                break;
        }

#endif

        ////////////////////////////////////////////////////////////////////////
        // Compute heading
        ////////////////////////////////////////////////////////////////////////

        if (target_set && !target_reached) {

            // Save current location to history
            emily_location_history[emily_location_history_pointer] = emily_location;

            // Update circular array pointer
            emily_location_history_pointer = (emily_location_history_pointer + 1) % EMILY_LOCATION_HISTORY_SIZE;
        }

        // Average of all the angles from points in history to current angle
        ////////////////////////////////////////////////////////////////////////

        //                double angle_sum = 0;
        //        
        //                for (int i = 0; i < EMILY_LOCATION_HISTORY_SIZE; i++) {
        //                    // If the coordinates are not zero
        //                    if (emily_location.x != 0 && emily_location.y != 0 && emily_location_history[i].x != 0 && emily_location_history[i].y != 0) {
        //        
        //                        // Difference in x axis
        //                        int delta_x = emily_location.x - emily_location_history[i].x;
        //        
        //                        // Difference in y axis
        //                        int delta_y = emily_location.y - emily_location_history[i].y;
        //        
        //                        // Angle in degrees
        //                        double angle = atan2(delta_y, delta_x) * (180 / M_PI);
        //        
        //                        angle_sum += angle;
        //                    }
        //                }
        //                
        //                // Angle average
        //                double angle = angle_sum / EMILY_LOCATION_HISTORY_SIZE;
        //        
        //                // Compute heading point
        //                Point heading_point;
        //                heading_point.x = (int) round(emily_location.x + HEADING_LINE_LENGTH * cos(angle * CV_PI / 180.0));
        //                heading_point.y = (int) round(emily_location.y + HEADING_LINE_LENGTH * sin(angle * CV_PI / 180.0));
        //        
        //                // Draw line between current location and heading point
        //                line(original_frame, emily_location, heading_point, Scalar(0, 0, 255), 1, 8, 0);
        //        
        //                // Draw line between historical location and current location
        //                line(original_frame, emily_location, emily_location_history[emily_location_history_pointer], Scalar(255, 0, 0), 1, 8, 0);

        // Just the oldest point in history
        ////////////////////////////////////////////////////////////////////////

        // If the coordinates are not zero
        if (emily_location.x != 0 && emily_location.y != 0 && emily_location_history[emily_location_history_pointer].x != 0 && emily_location_history[emily_location_history_pointer].y != 0) {

            //            // Difference in x axis
            //            int delta_x = emily_location.x - emily_location_history[emily_location_history_pointer].x;
            //
            //            // Difference in y axis
            //            int delta_y = emily_location.y - emily_location_history[emily_location_history_pointer].y;
            //
            //            // Angle in degrees
            //            emily_angle = atan2(delta_y, delta_x) * (180 / M_PI);
            //
            //            // Print angle to console
            //            //cout << emily_angle << endl;
            //
            //            // Compute heading point
            //            Point heading_point;
            //            heading_point.x = (int) round(emily_location.x + HEADING_LINE_LENGTH * cos(emily_angle * CV_PI / 180.0));
            //            heading_point.y = (int) round(emily_location.y + HEADING_LINE_LENGTH * sin(emily_angle * CV_PI / 180.0));
            //
            //            // Draw line between current location and heading point
            //            line(original_frame, emily_location, heading_point, Scalar(255, 0, 0), HEADING_LINE_THICKNESS, 8, 0);
            //
            //            // Draw line between historical location and current location
            //            line(original_frame, emily_location, emily_location_history[emily_location_history_pointer], Scalar(255, 0, 0), HEADING_LINE_THICKNESS, 8, 0);

            // Fitting a polynomial curve
            ////////////////////////////////////////////////////////////////////////

            // Initialize curve
            vector<Point> path_polynomial_approximation;

            // Sort EMILY location history chronologically
            Point emily_location_history_sorted[EMILY_LOCATION_HISTORY_SIZE];
            for (int i = 0; i < EMILY_LOCATION_HISTORY_SIZE; i++) {
                emily_location_history_sorted[i] = emily_location_history[(emily_location_history_pointer + i) % EMILY_LOCATION_HISTORY_SIZE];
            }

            // Initialize input vector (approxPolyDP takes only vectors and not arrays)
            vector<Point> input_points(emily_location_history_sorted, emily_location_history_sorted + sizeof emily_location_history_sorted / sizeof emily_location_history_sorted[0]);

            // Approximate location history with a polynomial curve
            approxPolyDP(input_points, path_polynomial_approximation, 4, false);

            // Draw polynomial curve
            for (int i = 0; i < path_polynomial_approximation.size() - 1; i++) {
                line(original_frame, path_polynomial_approximation[i], path_polynomial_approximation[i + 1], Scalar(255, 0, 255), HEADING_LINE_THICKNESS, CV_AA);
            }

            // Difference in x axis
            int delta_x_curve = path_polynomial_approximation[path_polynomial_approximation.size() - 1].x - path_polynomial_approximation[path_polynomial_approximation.size() - 2].x;

            // Difference in y axis
            int delta_Y_curve = path_polynomial_approximation[path_polynomial_approximation.size() - 1].y - path_polynomial_approximation[path_polynomial_approximation.size() - 2].y;

            // Angle in degrees
            double emily_angle_polynomial_approximation = atan2(delta_Y_curve, delta_x_curve) * (180 / M_PI);

            // Compute heading point
            Point heading_point_polynomial_approximation;
            heading_point_polynomial_approximation.x = (int) round(path_polynomial_approximation[path_polynomial_approximation.size() - 1].x + HEADING_LINE_LENGTH * cos(emily_angle_polynomial_approximation * CV_PI / 180.0));
            heading_point_polynomial_approximation.y = (int) round(path_polynomial_approximation[path_polynomial_approximation.size() - 1].y + HEADING_LINE_LENGTH * sin(emily_angle_polynomial_approximation * CV_PI / 180.0));

            // Draw line between current location and heading point
            line(original_frame, emily_location, heading_point_polynomial_approximation, Scalar(255, 255, 0), HEADING_LINE_THICKNESS, 8, 0);

            // Use curve polynomial tangent angle
            emily_angle = emily_angle_polynomial_approximation;
        }

        ////////////////////////////////////////////////////////////////////////
        // Show results
        ////////////////////////////////////////////////////////////////////////

        // Simple blob detector
        ////////////////////////////////////////////////////////////////////////

        //        drawKeypoints(original_frame, keypoints, original_frame, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //        
        //        imshow("Simple Blob Detector", original_frame);

        // Contours
        ////////////////////////////////////////////////////////////////////////

        //        for(int contour = 0; contour >= 0; contour = hierarchy[contour][0]) {
        //            drawContours(original_frame, contours, contour, Scalar(255, 0, 0), CV_FILLED, 8, hierarchy);
        //        }
        //        
        //        imshow("Countours", original_frame);

        // Threshold
        ////////////////////////////////////////////////////////////////////////

        //        imshow("Threshold", threshold);

        // Original image
        ////////////////////////////////////////////////////////////////////////

        //        imshow("Original image", original_frame);

        // Main Window
        ////////////////////////////////////////////////////////////////////////

        // Show target location
        if (target_location.x != 0 && target_location.y != 0) {
            circle(original_frame, target_location, TARGET_RADIUS - 1, TARGET_COLOR, 1, 8, 0);
            line(original_frame, Point(target_location.x - (TARGET_RADIUS / 2), target_location.y + (TARGET_RADIUS / 2)), Point(target_location.x + (TARGET_RADIUS / 2), target_location.y - (TARGET_RADIUS / 2)), TARGET_COLOR, 1, 8, 0);
            line(original_frame, Point(target_location.x - (TARGET_RADIUS / 2), target_location.y - (TARGET_RADIUS / 2)), Point(target_location.x + (TARGET_RADIUS / 2), target_location.y + (TARGET_RADIUS / 2)), TARGET_COLOR, 1, 8, 0);
        }

#ifndef CAMSHIFT        

        // Get size of each frame to be displayed
        Size blured_frame_size = blured_frame.size();
        Size threshold_size = threshold.size();
        Size eroded_dilated_threshold_size = eroded_dilated_threshold.size();
        Size original_frame_size = original_frame.size();

        // Convert threshold to RGB color space
        Mat threshold_color;
        cvtColor(threshold, threshold_color, CV_GRAY2RGB);

        // Convert eroded dilated threshold to RGB color space
        Mat eroded_dilated_threshold_color;
        cvtColor(eroded_dilated_threshold, eroded_dilated_threshold_color, CV_GRAY2RGB);

#ifndef FOUR_FRAME_MODE

        // Initialize output frame for 2 frames
        Mat output(threshold_size.height, threshold_size.width + original_frame_size.width, original_frame.type());

        // Initialize helper frames for 2 frames
        Mat threshold_new(output, Rect(0, 0, threshold_size.width, threshold_size.height));
        Mat original_frame_new(output, Rect(threshold_size.width, 0, original_frame_size.width, original_frame_size.height));

        // Copy frames to helper frames for 2 frames
        threshold_color.copyTo(threshold_new);
        original_frame.copyTo(original_frame_new);

#else

        // Initialize output frame for 4 frames
        Mat output(blured_frame_size.height + eroded_dilated_threshold_size.height, blured_frame_size.width + threshold_size.width, original_frame.type());

        // Initialize helper frames for 4 frames
        Mat blured_frame_new(output, Rect(0, 0, blured_frame_size.width, blured_frame_size.height));
        Mat threshold_new(output, Rect(blured_frame_size.width, 0, threshold_size.width, threshold_size.height));
        Mat eroded_dilated_threshold_new(output, Rect(0, blured_frame_size.height, eroded_dilated_threshold_size.width, eroded_dilated_threshold_size.height));
        Mat original_frame_new(output, Rect(eroded_dilated_threshold_size.width, blured_frame_size.height, original_frame_size.width, original_frame_size.height));

        // Copy frames to helper frames for 4 frames
        blured_frame.copyTo(blured_frame_new);
        threshold_color.copyTo(threshold_new);
        eroded_dilated_threshold_color.copyTo(eroded_dilated_threshold_new);
        original_frame.copyTo(original_frame_new);

#endif

#else

        Mat output;
        original_frame.copyTo(output);

#endif

        // Show output frame in the main window
        imshow(MAIN_WINDOW, output);

        // Set main window to full screen
        setWindowProperty(MAIN_WINDOW, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

        ////////////////////////////////////////////////////////////////////////
        // Video output
        ////////////////////////////////////////////////////////////////////////

        // Write the frame to the output video
        output_video << original_frame;
        
        ////////////////////////////////////////////////////////////////////////
        // Quantitative analysis
        ////////////////////////////////////////////////////////////////////////
        
        #ifdef ANALYSIS
        
        // The objective of this test is to select EMILY and then keep the
        // cursor in EMILY's center. The program will compute distance error
        // between the cursor and tracked location.
        
        // Only compute the error if object is selected
        
        #ifdef CAMSHIFT
            if (object_selected) {
        #endif
            
            // Compute distance of EMILY from the cursor
            double distance_error = sqrt(pow(emily_location.x - mouse_location.x, 2) + pow(emily_location.y - mouse_location.y, 2));
        
            // Log the distance error
            error_log_file << distance_error << endl;

        #ifdef CAMSHIFT
            }
        #endif
            
        #endif
            
        ////////////////////////////////////////////////////////////////////////
        // Control
        ////////////////////////////////////////////////////////////////////////

        commands current_commands;

        // If target was reached, clear the history
        if (target_reached) {

            // Set each element in history to 0
            for (int i = 0; i < EMILY_LOCATION_HISTORY_SIZE; i++) {
                emily_location_history[i] = Point(0, 0);
            }

        }

        bool heading_known = emily_location.x != 0 && emily_location.y != 0 && emily_location_history[emily_location_history_pointer].x != 0 && emily_location_history[emily_location_history_pointer].y != 0;


        // If the object is selected, begin control
        if (object_selected && target_set && !target_reached) {

            if (!heading_known) {

                current_commands.throttle = 0.2;
                current_commands.rudder = 0;

            } else {

                // Get rudder and throttle
                current_commands = get_control_commands(emily_location.x, emily_location.y, emily_angle, target_location.x, target_location.y);

            }

        } else {

            current_commands.throttle = 0;
            current_commands.rudder = 0;

        }

        // Debugging
        cout << "Throttle: " << current_commands.throttle << " Rudder: " << current_commands.rudder << endl;

        // Log throttle
        throttle_log_file << current_commands.throttle << endl;

        // Log rudder
        rudder_log_file << current_commands.rudder << endl;

        ////////////////////////////////////////////////////////////////////////
        // Communication
        ////////////////////////////////////////////////////////////////////////

        // Pack datagram
        double package[2];
        package[0] = current_commands.throttle;
        package[1] = current_commands.rudder;

        // Send throttle
        sendto(socket_descriptor, &package, 2 * sizeof (double), 0, (struct sockaddr *) &socket_address, sizeof (socket_address));

        ////////////////////////////////////////////////////////////////////////
        // Log output
        ////////////////////////////////////////////////////////////////////////

        // Get current time
        time_t raw_time;
        time(&raw_time);
        struct tm * local_time;
        local_time = localtime(&raw_time);
        char current_time[40];
        strftime(current_time, 40, "%Y%m%d%H%M%S", local_time);

        // Log time
        log_file << current_time;
        log_file << " ";

        // Log EMILY location
        log_file << emily_location.x;
        log_file << " ";
        log_file << emily_location.y;
        log_file << " ";

        // Log EMILY pose line segment
        log_file << emily_pose_point_1.x;
        log_file << " ";
        log_file << emily_pose_point_1.y;
        log_file << " ";
        log_file << emily_pose_point_2.x;
        log_file << " ";
        log_file << emily_pose_point_2.y;
        log_file << " ";

        // Log target location
        log_file << target_location.x;
        log_file << " ";
        log_file << target_location.y;
        log_file << " ";
        
        // Log EMILY angle
        log_file << emily_angle;
        log_file << " ";
        
        // Log distance to target
        log_file << current_commands.distance_to_target;
        log_file << " ";
        
        // Log error angle to target
        log_file << current_commands.angle_error_to_target;
        log_file << " ";
        
        // Log throttle
        log_file << current_commands.throttle;
        log_file << " ";
        
        // Log rudder
        log_file << current_commands.rudder;
        log_file << "\n";
    }

    // Close log
    log_file.close();
    rudder_log_file.close();

    // Close socket
    close(socket_descriptor);

    // Announce that the processing was finished
    cout << "Processing finished!" << endl;

    // Clean return
    return 0;
}