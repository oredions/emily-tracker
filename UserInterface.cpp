/* 
 * File:   UserInterface.cpp
 * Author: Jan Dufek
 */

#include "UserInterface.hpp"

#define CAMSHIFT

// Program settings
Settings * UserInterface::settings;

// Original point of click
Point UserInterface::origin;

Size UserInterface::video_size;

UserInterface::UserInterface(Settings& s, Size sz) {

    UserInterface::settings = &s;

    UserInterface::video_size = sz;

    // Show main window including slide bars
    create_main_window();

#ifdef CAMSHIFT

    // Histogram window
    namedWindow("Histogram", 0);

#endif

    // Set mouse handler on main window to choose object of interest
    setMouseCallback(UserInterface::settings->MAIN_WINDOW, onMouse, 0);

    // Set main window to full screen
    setWindowProperty(settings->MAIN_WINDOW, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
}

UserInterface::UserInterface(const UserInterface& orig) {
}

UserInterface::~UserInterface() {
}

/**
 * Trackbar handler. Called when track bar is clicked on.
 * 
 */
void UserInterface::on_trackbar(int, void*) {

    // Gaussian kernel size must be positive and odd. Or, it can be zero’s and
    // then it is computed from sigma.
    if (UserInterface::settings->blur_kernel_size % 2 == 0) {
        UserInterface::settings->blur_kernel_size++;
    }

    // Erode size cannot be 0
    if (UserInterface::settings->erode_size == 0) {
        UserInterface::settings->erode_size++;
    }

    // Dilate size cannot be 0
    if (UserInterface::settings->dilate_size == 0) {
        UserInterface::settings->dilate_size++;
    }
}

/**
 * Shows the main window and creates track bars.
 * 
 */
void UserInterface::create_main_window() {

    // Show new window
    namedWindow(UserInterface::settings->MAIN_WINDOW, CV_GUI_NORMAL);

#ifndef CAMSHIFT

    // Hue 1 trackbars
    createTrackbar("H 1 Min", MAIN_WINDOW, &hue_1_min, 180, on_trackbar);
    createTrackbar("H 1 Max", MAIN_WINDOW, &hue_1_max, 180, on_trackbar);

    // Hue 2 trackbars
    createTrackbar("H 2 Min", MAIN_WINDOW, &hue_2_min, 180, on_trackbar);
    createTrackbar("H 2 Max", MAIN_WINDOW, &hue_2_max, 180, on_trackbar);

#endif

    // Saturation trackbars
    createTrackbar("S Min", UserInterface::settings->MAIN_WINDOW, &UserInterface::settings->saturation_min, 255, on_trackbar);
    createTrackbar("S Max", UserInterface::settings->MAIN_WINDOW, &UserInterface::settings->saturation_max, 255, on_trackbar);

    // Value trackbars
    createTrackbar("V Min", UserInterface::settings->MAIN_WINDOW, &UserInterface::settings->value_min, 255, on_trackbar);
    createTrackbar("V Max", UserInterface::settings->MAIN_WINDOW, &UserInterface::settings->value_max, 255, on_trackbar);

    // Gaussian blur trackbar
    createTrackbar("Blur", UserInterface::settings->MAIN_WINDOW, &UserInterface::settings->blur_kernel_size, min(UserInterface::video_size.height, UserInterface::video_size.width), on_trackbar);

    // Target reached radius trackbar
    createTrackbar("Radius", UserInterface::settings->MAIN_WINDOW, &UserInterface::settings->target_radius, min(UserInterface::video_size.height, UserInterface::video_size.width), on_trackbar);

    // Proportional controller trackbar
    createTrackbar("Proportion", UserInterface::settings->MAIN_WINDOW, &UserInterface::UserInterface::settings->proportional, 100, on_trackbar);

    // Camera angle trackbar
    createTrackbar("Angle", UserInterface::UserInterface::settings->MAIN_WINDOW, &UserInterface::UserInterface::settings->camera_angle_degrees, 90, on_trackbar);

#ifndef CAMSHIFT    

    // Erode size trackbar
    createTrackbar("Erode", MAIN_WINDOW, &erode_size, min(UserInterface::video_size.height, UserInterface::video_size.width), on_trackbar);

    // Dilate size trackbar
    createTrackbar("Dilate", MAIN_WINDOW, &dilate_size, min(UserInterface::video_size.height, UserInterface::video_size.width), on_trackbar);

#endif    

}

/**
 * Mouse handler.
 * 
 */
void UserInterface::onMouse(int event, int x, int y, int flags, void*) {

    // Select object mode
    if (select_object) {

        // Get selected rectangle
        selection.x = MIN(x, UserInterface::origin.x);
        selection.y = MIN(y, UserInterface::origin.y);
        selection.width = abs(x - UserInterface::origin.x);
        selection.height = abs(y - UserInterface::origin.y);

        // Get intersection with the original image
        selection &= Rect(0, 0, UserInterface::video_size.width, UserInterface::video_size.height); // TODO check if works
    }

    // Check mouse button
    switch (event) {

            // Right drag is reserved for moving over the image
            // Left click context menu is disabled
            // Scrool is reserved for zoom

            // Right drag to select EMILY
        case EVENT_RBUTTONDOWN:

            // Save current point as click origin
            UserInterface::origin = Point(x, y);

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
void UserInterface::draw_position(int x, int y, double radius, Mat &frame) {

#ifndef CAMSHIFT

    // Circle
    circle(frame, Point(x, y), radius, UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);

#endif

    // Lines
    if (y - radius > 0) {
        line(frame, Point(x, y), Point(x, y - radius), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(x, 0), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    }

    if (y + radius < UserInterface::video_size.height) {
        line(frame, Point(x, y), Point(x, y + radius), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(x, UserInterface::video_size.height), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    }

    if (x - radius > 0) {
        line(frame, Point(x, y), Point(x - radius, y), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(0, y), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    }

    if (x + radius < UserInterface::video_size.width) {
        line(frame, Point(x, y), Point(x + radius, y), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    } else {
        line(frame, Point(x, y), Point(UserInterface::video_size.width, y), UserInterface::settings->LOCATION_COLOR, UserInterface::settings->LOCATION_THICKNESS);
    }

    // Text coordinates
    putText(frame, "[" + int_to_string(x) + "," + int_to_string(y) + "]", Point(x, y + radius + 20), 1, 1, UserInterface::settings->LOCATION_COLOR, 1, 8);
}

/**
 * Draws axis of given rotated rectangle. Axis is an principal axis of symmetry.
 * 
 * @param rectangle rotated rectangle for which to draw principal axis
 */
void UserInterface::draw_principal_axis(RotatedRect rectangle, Mat& frame) {

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
    line(frame, shortest_axis_midpoint_1, shortest_axis_midpoint_2, UserInterface::settings->POSE_LINE_COLOR, UserInterface::settings->POSE_LINE_THICKNESS, 8);

}

/**
 * Draws current target location.
 * 
 * @param frame
 * @param target_location
 */
void UserInterface::draw_target(Mat& frame, Point target_location) {
    if (target_location.x != 0 && target_location.y != 0) {
        circle(frame, target_location, settings->TARGET_RADIUS - 1, settings->TARGET_COLOR, 1, 8, 0);
        line(frame, Point(target_location.x - (settings->TARGET_RADIUS / 2), target_location.y + (settings->TARGET_RADIUS / 2)), Point(target_location.x + (settings->TARGET_RADIUS / 2), target_location.y - (settings->TARGET_RADIUS / 2)), settings->TARGET_COLOR, 1, 8, 0);
        line(frame, Point(target_location.x - (settings->TARGET_RADIUS / 2), target_location.y - (settings->TARGET_RADIUS / 2)), Point(target_location.x + (settings->TARGET_RADIUS / 2), target_location.y + (settings->TARGET_RADIUS / 2)), settings->TARGET_COLOR, 1, 8, 0);

        // Draw target acceptance radius
        circle(frame, target_location, settings->target_radius, settings->TARGET_COLOR, 1, 8, 0);
    }
}

/**
 * Prints current status to the GUI.
 * 
 * @param frame
 * @param status
 * @param time_to_target
 */
void UserInterface::print_status(Mat& frame, int status, double time_to_target) {

    String stringStatus;

    switch (status) {
        case 0:
            stringStatus = "Initialization";
            break;
        case 1:
            stringStatus = "Select EMILY and target.";
            break;
        case 2:
            stringStatus = "Target set. Getting orientation.";
            break;
        case 3:
            stringStatus = "Target set. Going to target.";
            break;
        case 4:

            // Covert time to target to string
            ostringstream stringStream;
            stringStream << time_to_target;
            std::string timeToTargetString = stringStream.str();

            stringStatus = "Target reached in " + timeToTargetString + " s";
            break;
    }

    // Print status
    putText(frame, stringStatus, Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
}

/**
 * Convert integer to string.
 * 
 * @param number integer to be converted to string
 * 
 */
string UserInterface::int_to_string(int number) {
    stringstream stringStream;
    stringStream << number;
    return stringStream.str();
}

/**
 * Show main window.
 * 
 * @param mat
 */
void UserInterface::show_main(Mat& mat) {
    imshow(UserInterface::settings->MAIN_WINDOW, mat);
}

/**
 * Show histogram window.
 * 
 * @param mat
 */
void UserInterface::show_histogram(Mat& mat) {
    imshow(UserInterface::settings->HISTOGRAM_WINDOW, mat);
}