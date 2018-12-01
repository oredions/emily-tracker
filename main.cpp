/** 
 * @file    main.cpp
 * @author  Jan Dufek
 * @date    07/01/2016
 * @version 2.0
 *  
 * This project uses the video from a small unmanned aerial system to
 * autonomously navigate an unmanned surface vehicle covered in a flotation
 * jacket to reach drowning victims.
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
#include "Settings.hpp"
#include "Control.hpp"
#include "OutputVideo.hpp"
#include "Logger.hpp"
#include "Communication.hpp"
#include "UserInterface.hpp"
#include "Undistort.hpp"
#include <sys/socket.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>

#define PI 3.14159265

////////////////////////////////////////////////////////////////////////////////
// TODOs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Name spaces
////////////////////////////////////////////////////////////////////////////////

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Settings
////////////////////////////////////////////////////////////////////////////////

Settings * settings = new Settings();

////////////////////////////////////////////////////////////////////////////////
// Video Capture
////////////////////////////////////////////////////////////////////////////////

VideoCapture video_capture = VideoCapture(settings->video_capture_source);

////////////////////////////////////////////////////////////////////////////////
// Control
////////////////////////////////////////////////////////////////////////////////

Control * control = new Control(* settings);

////////////////////////////////////////////////////////////////////////////////
// Algorithm
////////////////////////////////////////////////////////////////////////////////

// Enable advanced Camshift algorithm. It will let you choose object
// object of interest, computes histogram, and tracks it using mean shift.
// Important: if CAMSHIFT pragma is not defined, do not
// define WAIT_FOR_OBJECT_SELECTION pragma, as there would be no object selection.
#define CAMSHIFT

// Enable inverse perspective warping. This will take an angle from GUI and
// approximate overhead view.
//#define INVERSE_PERSPECTIVE_WARP

// This will wait for an object of interest to be selected before loading next
// frames. It will load the first frame only and wait for the user to select
// an object. After an object is selected, it will continue loading next frames.
#define WAIT_FOR_OBJECT_SELECTION

////////////////////////////////////////////////////////////////////////////////
// Analysis
////////////////////////////////////////////////////////////////////////////////

// Enables analysis of localization performance. As soon as an object is
// selected, it will log pixel distance from the mouse cursor. The operator
// have to keep the cursor in the centroid of EMILY all the time to mark its
// true position.
//#define ANALYSIS;

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

// Back projection mode toggle
bool back_projection_mode = false;

// Select object flag
bool select_object = false;

// Track object mode toggle
int object_selected = 0;

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
Point * emily_location_history = new Point[settings->EMILY_LOCATION_HISTORY_SIZE];

#ifdef ANALYSIS

Point mouse_location;

#endif

// EMILY motion angle
double emily_angle;

// Target was reached
bool target_reached = false;

// Target reached in this iteration
bool target_reached_now = false;

// Timer to estimate EMILY heading
int emily_location_history_pointer;

// EMILY pose
Point emily_pose_point_1;
Point emily_pose_point_2;

// Status of the algorithm
int status = 0;

// Time it takes to reach the target.
time_t startTarget, endTarget;
double timeToTarget = 0;

// Frame number
long frame_number = -1;

/**
 * Get size of the give rectangle. The size is measured as distance of midpoints
 * of shorter sides.
 * 
 * @param rectangle
 * @return 
 */
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
 * Draw axis give two points. This method is currently unused.
 * 
 * @param img
 * @param p Point 1
 * @param q Point 2
 * @param colour
 * @param scale
 */
void draw_axis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2) {
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
 * Get orientation given location history. This method is currently unused.
 * 
 * @param pts
 * @param img
 * @return 
 */
double get_orientation(const vector<Point> &pts, Mat &img) {

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
    draw_axis(img, cntr, p1, Scalar(0, 255, 0), 1);
    draw_axis(img, cntr, p2, Scalar(255, 255, 0), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians

    return angle;
}

/**
 * Get frame per seconds of the input video feed.
 * 
 * @return Frame per seconds
 */
double get_input_video_fps() {
    double input_video_fps = video_capture.get(CV_CAP_PROP_FPS);

    // If the input is video stream, we have to calculate FPS manually
    if (input_video_fps == 0) {

        // Number of sample frames to capture
        int num_sample_frames = 50;

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

    return input_video_fps;
}

/**
 * Get the resolution of the input video feed.
 */
void get_input_video_size() {

    Size input_video_size(video_capture.get(CV_CAP_PROP_FRAME_WIDTH), video_capture.get(CV_CAP_PROP_FRAME_HEIGHT));

    // If the input video exceeds processing video size limits, we will have to resize it
    if (input_video_size.height > settings->PROCESSING_VIDEO_HEIGHT_LIMIT) {

        // Compute scale ratio
        double ratio = (double) settings->PROCESSING_VIDEO_HEIGHT_LIMIT / input_video_size.height;

        // Compute new width
        int new_video_width = input_video_size.width * ratio;

        // Set new size
        resized_video_size.height = settings->PROCESSING_VIDEO_HEIGHT_LIMIT;
        resized_video_size.width = new_video_width;

        // Set parameter for maximum blob area
        settings->MAX_BLOB_AREA = resized_video_size.height * resized_video_size.width;

        // Indicate that resizing is necessary
        resize_video = true;

    } else {

        // Set video size to original size
        resized_video_size.height = input_video_size.height;
        resized_video_size.width = input_video_size.width;

        // Set parameter for maximum blob area
        settings->MAX_BLOB_AREA = resized_video_size.height * resized_video_size.width;

        // Indicate that resizing is not necessary
        resize_video = false;

    }
}

/**
 * Equalize histogram of the given frame.
 * 
 * @param HSV_frame
 */
void equalize(Mat& HSV_frame) {
    vector<Mat> HSV_planes;
    split(HSV_frame, HSV_planes);
    equalizeHist(HSV_planes[2], HSV_planes[2]);
    merge(HSV_planes, HSV_frame);
}

/**
 * Create histogram for the area of interest.
 * 
 * @param object_of_interest
 * @param histogram_size
 * @param pointer_histogram_ranges
 * @param hue
 * @param saturation_value_threshold
 * @param histogram
 * @param histogram_image
 */
void create_histogram(Rect& object_of_interest, int& histogram_size, const float*& pointer_histogram_ranges, Mat& hue, Mat& saturation_value_threshold, Mat& histogram, Mat& histogram_image) {

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

/**
 * Get orientation of the USV based on its location history.
 */
void get_orientation() {

    // Initialize curve
    vector<Point> path_polynomial_approximation;

    // Sort EMILY location history chronologically
    Point * emily_location_history_sorted = new Point[settings->EMILY_LOCATION_HISTORY_SIZE];

    // Initialize input vector (approxPolyDP takes only vectors and not arrays)
    vector<Point> input_points;

    for (int i = 0; i < settings->EMILY_LOCATION_HISTORY_SIZE; i++) {
        emily_location_history_sorted[i] = emily_location_history[(emily_location_history_pointer + i) % settings->EMILY_LOCATION_HISTORY_SIZE];
        input_points.push_back(emily_location_history_sorted[i]);
    }

    // Approximate location history with a polynomial curve
    approxPolyDP(input_points, path_polynomial_approximation, 4, false);

    // Draw polynomial curve
    for (int i = 0; i < path_polynomial_approximation.size() - 1; i++) {
        line(original_frame, path_polynomial_approximation[i], path_polynomial_approximation[i + 1], Scalar(255, 0, 255), settings->HEADING_LINE_THICKNESS, CV_AA);
    }

    // Difference in x axis
    int delta_x_curve = path_polynomial_approximation[path_polynomial_approximation.size() - 1].x - path_polynomial_approximation[path_polynomial_approximation.size() - 2].x;

    // Difference in y axis
    int delta_Y_curve = path_polynomial_approximation[path_polynomial_approximation.size() - 1].y - path_polynomial_approximation[path_polynomial_approximation.size() - 2].y;

    // Angle in degrees
    double emily_angle_polynomial_approximation = atan2(delta_Y_curve, delta_x_curve) * (180 / M_PI);

    // Compute heading point
    Point heading_point_polynomial_approximation;
    heading_point_polynomial_approximation.x = (int) round(path_polynomial_approximation[path_polynomial_approximation.size() - 1].x + settings->HEADING_LINE_LENGTH * cos(emily_angle_polynomial_approximation * CV_PI / 180.0));
    heading_point_polynomial_approximation.y = (int) round(path_polynomial_approximation[path_polynomial_approximation.size() - 1].y + settings->HEADING_LINE_LENGTH * sin(emily_angle_polynomial_approximation * CV_PI / 180.0));

    // Draw line between current location and heading point
    line(original_frame, emily_location, heading_point_polynomial_approximation, Scalar(255, 255, 0), settings->HEADING_LINE_THICKNESS, 8, 0);

    // Use curve polynomial tangent angle
    emily_angle = emily_angle_polynomial_approximation;
}

/**
 * Create one log entry with current system status.
 * 
 * @param logger
 * @param current_commands
 */
void create_log_entry(Logger* logger, Command* current_commands) {

    // Log throttle
    logger->log_throttle(current_commands->get_throttle());
    logger->log_throttle("\n");

    // Log rudder
    logger->log_rudder(current_commands->get_rudder());
    logger->log_rudder("\n");

    // Get current time
    time_t raw_time;
    time(&raw_time);
    struct tm * local_time;
    local_time = localtime(&raw_time);
    char current_time[40];
    strftime(current_time, 40, "%Y%m%d%H%M%S", local_time);

    // Log time
    logger->log_general(current_time);
    logger->log_general(" ");

    // Log frame number
    logger->log_general(frame_number);
    logger->log_general(" ");

    // Log EMILY location
    logger->log_general(emily_location.x);
    logger->log_general(" ");
    logger->log_general(emily_location.y);
    logger->log_general(" ");

    // Log EMILY pose line segment
    logger->log_general(emily_pose_point_1.x);
    logger->log_general(" ");
    logger->log_general(emily_pose_point_1.y);
    logger->log_general(" ");
    logger->log_general(emily_pose_point_2.x);
    logger->log_general(" ");
    logger->log_general(emily_pose_point_2.y);
    logger->log_general(" ");

    // Log target location
    logger->log_general(target_location.x);
    logger->log_general(" ");
    logger->log_general(target_location.y);
    logger->log_general(" ");

    // Log EMILY angle
    logger->log_general(emily_angle);
    logger->log_general(" ");

    // Log distance to target
    logger->log_general(current_commands->get_distance_to_target());
    logger->log_general(" ");

    // Log error angle to target
    logger->log_general(current_commands->get_angle_error_to_target());
    logger->log_general(" ");

    // Log throttle
    logger->log_general(current_commands->get_throttle());
    logger->log_general(" ");

    // Log rudder
    logger->log_general(current_commands->get_rudder());
    logger->log_general(" ");

    // Log status
    logger->log_general(status);
    logger->log_general(" ");

    // Log time to target
    logger->log_general(timeToTarget);
    logger->log_general("\n");
}

/**
 * Update USV location history.
 * 
 * @param target_set
 */
void update_history(bool target_set) {
    if (target_set && !target_reached) {

        // Save current location to history
        emily_location_history[emily_location_history_pointer] = emily_location;

        // Update circular array pointer
        emily_location_history_pointer = (emily_location_history_pointer + 1) % settings->EMILY_LOCATION_HISTORY_SIZE;
    }
}

/**
 * Show current object of interest selection in the GUI.
 */
void show_selection() {
    if (select_object && selection.width > 0 && selection.height > 0) {
        Mat roi(original_frame, selection);
        bitwise_not(roi, roi);
    }
}

/**
 * Autonomously navigate the USV based on the UAV video feed to reach the target.
 */
int main(int argc, char** argv) {

    ////////////////////////////////////////////////////////////////////////////
    // Output video initialization
    //////////////////////////////////////////////////////////////////////////// 

    // Get FPS of the input video
    double input_video_fps = get_input_video_fps();

    // Inogeni for some reason cannot correctly estimate the FPS.
    // Therefore we use FPS equal to 7 which is approximate frequency of this algorithm.
#ifdef INOGENI

    input_video_fps = 7;

#endif

    // Get the size of input video
    get_input_video_size();

    // Output video name. It is in format year_month_day_hour_minute_second.avi.
    time_t raw_time;
    time(&raw_time);
    struct tm * local_time;
    local_time = localtime(&raw_time);
    char output_file_name[40];
    strftime(output_file_name, 40, "output/%Y_%m_%d_%H_%M_%S", local_time);
    string output_file_name_string(output_file_name);

    OutputVideo * output_video = new OutputVideo(input_video_fps, resized_video_size, output_file_name_string);
    VideoWriter video_writer = output_video->get_video_writer();

    ////////////////////////////////////////////////////////////////////////////
    // Log
    ////////////////////////////////////////////////////////////////////////////

    Logger * logger = new Logger(output_file_name_string);

#ifdef ANALYSIS

    // Open error log file
    ofstream error_log_file;
    error_log_file.open(output_file_name_string + "_error.txt");

#endif

    ////////////////////////////////////////////////////////////////////////////
    // GUI
    ////////////////////////////////////////////////////////////////////////////

    UserInterface * user_interface = new UserInterface(* settings, resized_video_size);

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

#ifdef WAIT_FOR_OBJECT_SELECTION
    
    // This will prevent the algorithm from loading second frame if the first
    // frame was not used yet. Will be set to true after the algorithm uses the
    // first frame
    bool first_frame_used = false;
    
#endif
    
    ////////////////////////////////////////////////////////////////////////////
    // Initialization of communication
    ////////////////////////////////////////////////////////////////////////////

    Communication * communication = new Communication(settings->IP_ADDRESS, settings->PORT);

    ////////////////////////////////////////////////////////////////////////////
    // Initialization of camera distortion parameters
    ////////////////////////////////////////////////////////////////////////////

#ifdef INVERSE_PERSPECTIVE_WARP
    Undistort * undistort = new Undistort(* settings, resized_video_size);
#endif

    ////////////////////////////////////////////////////////////////////////////
    // Tracking
    ////////////////////////////////////////////////////////////////////////////

    //    // Load first frame
    //    Mat first;
    //    video_capture >> first;

#ifdef WAIT_FOR_OBJECT_SELECTION
    
    // Always read the first frame so that the object of interest can be
    // selected. First frame has to be stored in its own variable because the
    // algorithm draws into original_frame and therefore it cannot be reused
    // in the next iteration.
    Mat first_frame;
    video_capture >> first_frame;
    frame_number++;
    
#endif

    // Iterate over each frame from the video input and wait between iterations.
    while (waitKey(1) != 27) {

        // If not paused
        if (!paused) {

#ifdef WAIT_FOR_OBJECT_SELECTION
            
            // Only load new frames after object of interest was selected and is being tracked (-1 is only selected but not tracked yet, 0 is not selected at all)
            if (object_selected == 1) {

                // If the first frame was not used yet, do not load the second frame
                if (!first_frame_used) {

                    // Used the first frame (it is important to use copyTo,
                    // otherwise it will be assigned by reference and it will make the first frame dirty.
                    first_frame.copyTo(original_frame);
                    
                    // Now the first frame was used, so next time load the second frame
                    first_frame_used = true;

                } else {

                    // Read one frame
                    video_capture >> original_frame;

                    // Increase frame number counter
                    frame_number++;

                }

                // End if frame is empty
                if (original_frame.empty()) {
                    break;
                }
                
            } else {
             
                // Object is not selected so still use the first frame
                first_frame.copyTo(original_frame);
                
            }
            
#else

            // Read one frame
            video_capture >> original_frame;

            // Increase frame number counter
            frame_number++;

            // End if frame is empty
            if (original_frame.empty()) {
                break;
            }

#endif

        }

        ////////////////////////////////////////////////////////////////////////
        // Global flags
        ////////////////////////////////////////////////////////////////////////

        bool target_set = target_location.x != 0 && target_location.y != 0;

        ////////////////////////////////////////////////////////////////////////
        // Preprocessing
        ////////////////////////////////////////////////////////////////////////

        if (resize_video) {

            // Resize the input
            // TODO enable resize
            //resize(original_frame, original_frame, resized_video_size, 0, 0, INTER_LANCZOS4);

        }

        // Apply Gaussian blur filter
        GaussianBlur(original_frame, blured_frame, Size(settings->blur_kernel_size, settings->blur_kernel_size), 0, 0);

        // Convert to HSV color space
        Mat HSV_frame;
        cvtColor(blured_frame, HSV_frame, COLOR_BGR2HSV);

        // Equalize on value (V)
        equalize(HSV_frame);

        ////////////////////////////////////////////////////////////////////////
        // Distortion and Inverse Perspective Warping
        ////////////////////////////////////////////////////////////////////////

#ifdef INVERSE_PERSPECTIVE_WARP

        // Undistort camera
        // TODO enable undistort
        //undistort->undistort_camera(HSV_frame, original_frame);

        // Camera projection matrix
        // TODO change to automatic undistort
        undistort->undistort_perspective_manual(HSV_frame, original_frame);

#endif

        ////////////////////////////////////////////////////////////////////////
        // Thresholding
        ////////////////////////////////////////////////////////////////////////

#ifndef CAMSHIFT        

        // Threshold on lower red
        Mat lower_red_threshold;
        inRange(HSV_frame, cv::Scalar(settings->hue_1_min, settings->saturation_min, settings->value_min), cv::Scalar(settings->hue_1_max, settings->saturation_max, settings->value_max), lower_red_threshold);

        // Threshold on upper red
        Mat upper_red_threshold;
        inRange(HSV_frame, cv::Scalar(settings->hue_2_min, settings->saturation_min, settings->value_min), cv::Scalar(settings->hue_2_max, settings->saturation_max, settings->value_max), upper_red_threshold);

        // Add thresholds together
        Mat threshold;
        addWeighted(lower_red_threshold, 1.0, upper_red_threshold, 1.0, 0.0, threshold);

        // Erode to filter noise
        Mat eroded_dilated_threshold;
        Mat erode_element = getStructuringElement(MORPH_RECT, Size(settings->erode_size, settings->erode_size));
        erode(threshold, eroded_dilated_threshold, erode_element);
        erode(eroded_dilated_threshold, eroded_dilated_threshold, erode_element);

        // Dilate to make blobs more distinctive
        Mat dilate_element = getStructuringElement(MORPH_RECT, Size(settings->dilate_size, settings->dilate_size));
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
                if (area > settings->MIN_BLOB_AREA && area < settings->MAX_BLOB_AREA) {

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
                    user_interface->draw_principal_axis(min_ellipse, original_frame);

                    ////////////////////////////////////////////////////////////
                    // Draw EMILY location in the image
                    ////////////////////////////////////////////////////////////

                    // Compute object size
                    double object_size = get_size(min_ellipse);

                    // Draw object
                    user_interface->draw_position(max_area_object_x, max_area_object_y, object_size, original_frame);

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
                inRange(HSV_frame, Scalar(0, settings->saturation_min, settings->value_min), Scalar(180, settings->saturation_max, settings->value_max), saturation_value_threshold);

                // Mix channels
                int chanels[] = {0, 0};
                hue.create(HSV_frame.size(), HSV_frame.depth());
                mixChannels(&HSV_frame, 1, &hue, 1, chanels, 1);

                // Object does not have histogram yet, so create it
                if (object_selected < 0) {

                    // Create histogram of region of interest
                    create_histogram(object_of_interest, histogram_size, pointer_histogram_ranges, hue, saturation_value_threshold, histogram, histogram_image);

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
                    ellipse(original_frame, tracking_box, settings->LOCATION_COLOR, settings->LOCATION_THICKNESS, LINE_AA);

                    // Draw cross hairs
                    user_interface->draw_position(tracking_box.center.x, tracking_box.center.y, min(tracking_box.size.width, tracking_box.size.height) / 2, original_frame);

                    // Draw pose
                    user_interface->draw_principal_axis(tracking_box, original_frame);

                    // Save EMILY location
                    emily_location = Point(tracking_box.center.x, tracking_box.center.y);

                }

            }
        } else if (object_selected < 0) {

            // Un pause if the selection has been made
            paused = false;
        }

        // Show the selection
        show_selection();

        // Show the histogram
        user_interface->show_histogram(histogram_image);

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

        update_history(target_set);

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

            // Get orientation of EMILY
            get_orientation();

        }

        ////////////////////////////////////////////////////////////////////////
        // Show results
        ////////////////////////////////////////////////////////////////////////

        // Simple blob detector
        ////////////////////////////////////////////////////////////////////////

        //        drawKeypoints(original_frame, keypoints, original_frame, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //        
        //        userInterface->show_main(original_frame);

        // Contours
        ////////////////////////////////////////////////////////////////////////

        //        for(int contour = 0; contour >= 0; contour = hierarchy[contour][0]) {
        //            drawContours(original_frame, contours, contour, Scalar(255, 0, 0), CV_FILLED, 8, hierarchy);
        //        }
        //        
        //        userInterface->show_main(original_frame);

        // Threshold
        ////////////////////////////////////////////////////////////////////////

        //        userInterface->show_main(threshold);

        // Original image
        ////////////////////////////////////////////////////////////////////////

        //        userInterface->show_main(original_frame);

        // Main Window
        ////////////////////////////////////////////////////////////////////////

        // Show target location
        user_interface->draw_target(original_frame, target_location);

        // Get status as a string message
        user_interface->print_status(original_frame, status, timeToTarget);

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

        // Copy the frame to output
        Mat output;
        original_frame.copyTo(output);

#endif

        // Show output frame in the main window
        user_interface->show_main(output);

        ////////////////////////////////////////////////////////////////////////
        // Video output
        ////////////////////////////////////////////////////////////////////////

        // Write the frame to the output video
        video_writer << original_frame;
        //video_writer << threshold_color;

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

        // Initialize current commands object
        Command * current_commands = new Command();

        // If target was reached, clear the history
        if (target_reached) {

            // Set each element in history to 0
            for (int i = 0; i < settings->EMILY_LOCATION_HISTORY_SIZE; i++) {
                emily_location_history[i] = Point(0, 0);
            }

            // Set status
            status = 4;

            // Target was reached for the first time
            if (target_reached_now == true) {

                // End timer
                time(&endTarget);

                // Compute elapsed time
                timeToTarget = difftime(endTarget, startTarget);

                // Reset the flag
                target_reached_now = false;

            }

        }

        // Is the current heading known
        bool heading_known = emily_location.x != 0 && emily_location.y != 0 && emily_location_history[emily_location_history_pointer].x != 0 && emily_location_history[emily_location_history_pointer].y != 0;

        // If the object is selected, begin control
        if (object_selected && target_set && !target_reached) {

            if (!heading_known) {

                current_commands->set_throttle(0.2);
                current_commands->set_rudder(0);

                // Set status
                status = 2;

                // Start timer
                time(&startTarget);

            } else {

                // Get rudder and throttle
                current_commands = control->get_control_commands(emily_location.x, emily_location.y, emily_angle, target_location.x, target_location.y);

                // Set status
                status = 3;

                // Next time we reach the target, it is going to be for the first time
                target_reached_now = true;

            }

        } else {

            // Stop USV
            current_commands->set_throttle(0);
            current_commands->set_rudder(0);

            if (!target_reached) {

                // Set status
                status = 1;

            }

        }

        ////////////////////////////////////////////////////////////////////////
        // Communication
        ////////////////////////////////////////////////////////////////////////

        communication->send_command(* current_commands);

        ////////////////////////////////////////////////////////////////////////
        // Log output
        ////////////////////////////////////////////////////////////////////////

        // Debugging
        //cout << "Throttle: " << current_commands->get_throttle() << " Rudder: " << current_commands->get_rudder() << endl;

#ifdef WAIT_FOR_OBJECT_SELECTION
        
        // Only create log entry after the object been selected and first frame used for tracking
        if (object_selected && first_frame_used) {
            
#endif         
               
        // Log the data
        create_log_entry(logger, current_commands);
        
#ifdef WAIT_FOR_OBJECT_SELECTION
        
        }
            
#endif        
        
    }

    // Close logs
    delete logger;

    // Stop EMILY and close communication
    delete communication;

    // Announce that the processing was finished
    cout << "Processing finished!" << endl;

    // Clean return
    return 0;
}