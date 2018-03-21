/* 
 * File:   Undistort.cpp
 * Author: Jan Dufek
 */

#include "Undistort.hpp"

Undistort::Undistort(Settings& s, Size sz) {

    settings = &s;
    video_size = sz;

    initUndistortRectifyMap(settings->camera_intrinsic_matrix, settings->camera_distortion_vector, Mat(), getOptimalNewCameraMatrix(settings->camera_intrinsic_matrix, settings->camera_distortion_vector, video_size, 1, video_size, 0), video_size, CV_16SC2, undistortRectifyMap1, undistortRectifyMap2);

}

Undistort::Undistort(const Undistort& orig) {
}

Undistort::~Undistort() {
}

void Undistort::undistort_camera(Mat& HSV_frame, Mat& original_frame) {
    remap(original_frame, original_frame, undistortRectifyMap1, undistortRectifyMap2, INTER_LINEAR);
    remap(HSV_frame, HSV_frame, undistortRectifyMap1, undistortRectifyMap2, INTER_LINEAR);

    // Compute camera angle in radians
    settings->camera_angle_radians = ((double) settings->camera_angle_degrees - 90.) * PI / 180;
}

void Undistort::undistort_perspective(Mat& HSV_frame, Mat& original_frame) {
    Mat camera_projection_matrix = (Mat_<double>(4, 3) <<
            1, 0, 0,
            0, 1, 0,
            0, 0, 0,
            0, 0, 1);

    // Camera rotation matrix
    Mat camera_rotation_matrix = (Mat_<double>(4, 4) <<
            1, 0, 0, 0,
            0, cos(settings->camera_angle_radians), -sin(settings->camera_angle_radians), 0,
            0, sin(settings->camera_angle_radians), cos(settings->camera_angle_radians), 0,
            0, 0, 0, 1);

    // Camera translation matrix
    Mat camera_translation_matrix = (Mat_<double>(4, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 1,
            0, 0, 0, 1);

    // Camera intrinsic matrix 
    Mat camera_intrinsic_matrix_3D;
    hconcat(settings->camera_intrinsic_matrix, Mat(3, 1, CV_64F, double(0)), camera_intrinsic_matrix_3D);

    // Overall camera transformation matrix
    Mat camera_transformation = (camera_intrinsic_matrix_3D * (camera_translation_matrix * (camera_rotation_matrix * (camera_projection_matrix))));

    // Overall camera transformation inverse
    Mat camera_transformation_inverse;
    invert(camera_transformation, camera_transformation_inverse);

    // Get new boundaries of the frame after the application of the transformation

    // Top left corner
    Mat original_top_left = (Mat_<double>(3, 1) << 0, 0, 1);
    Mat new_top_left = camera_transformation_inverse * original_top_left;
    double new_top_left_x = new_top_left.at<double>(0, 0) / new_top_left.at<double>(2, 0);
    double new_top_left_y = new_top_left.at<double>(1, 0) / new_top_left.at<double>(2, 0);

    // Top right corner
    Mat original_top_right = (Mat_<double>(3, 1) << video_size.width, 0, 1);
    Mat new_top_right = camera_transformation_inverse * original_top_right;
    double new_top_right_x = new_top_right.at<double>(0, 0) / new_top_right.at<double>(2, 0);
    double new_top_right_y = new_top_right.at<double>(1, 0) / new_top_right.at<double>(2, 0);

    // Bottom left
    Mat original_bottom_left = (Mat_<double>(3, 1) << 0, video_size.height, 1);
    Mat new_bottom_left = camera_transformation_inverse * original_bottom_left;
    double new_bottom_left_x = new_bottom_left.at<double>(0, 0) / new_bottom_left.at<double>(2, 0);
    double new_bottom_left_y = new_bottom_left.at<double>(1, 0) / new_bottom_left.at<double>(2, 0);

    // Bottom right corner
    Mat original_bottom_right = (Mat_<double>(3, 1) << video_size.width, video_size.height, 1);
    Mat new_bottom_right = camera_transformation_inverse * original_bottom_right;
    double new_bottom_right_x = new_bottom_right.at<double>(0, 0) / new_bottom_right.at<double>(2, 0);
    double new_bottom_right_y = new_bottom_right.at<double>(1, 0) / new_bottom_right.at<double>(2, 0);

    // Compute scale for each axis so that the image fits the screen
    double scale_x = video_size.width / abs(new_top_right_x - new_top_left_x);
    double scale_y = video_size.height / abs(new_bottom_left_y - new_top_left_y);

    // Take overall scale as the largest scale
    double scale_overall = scale_x > scale_y ? scale_x : scale_y;

    // Scale matrix (scales the frame so that it fits the screen)
    Mat scale = (Mat_<double>(3, 3) <<
            scale_overall, 0, 0,
            0, scale_overall, 0,
            0, 0, 1);
    invert(scale, scale);

    // Translation matrix (translate the frame so that it starts at the bottom)
    Mat translate = (Mat_<double>(3, 3) <<
            // TODO there is a problem with video beeing shifted, this is quick fix
            //1, 0, (video_size.width / 2) + 200,
            1, 0, video_size.width / 2,
            0, 1, video_size.height - (new_bottom_left_y * scale_y),
            0, 0, 1);
    invert(translate, translate);

    // Redefine overall camera transformation matrix with scale and translation
    camera_transformation = (camera_intrinsic_matrix_3D * (camera_translation_matrix * (camera_rotation_matrix * (camera_projection_matrix * (scale * (translate))))));

    // Apply inverse perspective warp using the camera transformation matrix
    warpPerspective(original_frame, original_frame, camera_transformation, video_size, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
    warpPerspective(HSV_frame, HSV_frame, camera_transformation, video_size, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
}

/**
 * Manual undistort functionality.
 * 
 * @param HSV_frame
 * @param original_frame
 */
void Undistort::undistort_perspective_manual(Mat& HSV_frame, Mat& original_frame) {

    // TODO turn off resize

    //    // Manually load image for testing
    //    original_frame = imread("frame.png", CV_LOAD_IMAGE_COLOR);

    // Camera instrinsic matrix (sources:
    // https://www.dji.com/phantom-3-pro/info#specs and
    // https://forum.dji.com/thread-41515-1-1.html)
    double f = 3.61; // Focal length in milimeters
    double sensorWidth = 6.16;
    double sensorHeight = 4.62;
    double w = original_frame.cols; // Image width in pixels
    double h = original_frame.rows; // Image height in pixels
    double m_x = w / sensorWidth; // Number of pixels per unit distance in the X_i direction (px/mm)
    double m_y = h / sensorHeight; // Number of pixels per unit distance in the Y_i direction (px/mm)
    double alpha_x = f * m_x;
    double alpha_y = f * m_y;
    double x_0 = w / 2.0;
    double y_0 = h / 2.0;
    Mat K = (Mat_<double>(3, 4) <<
            alpha_x, 0, x_0, 0,
            0, alpha_y, y_0, 0,
            0, 0, 1, 0);

    // Rotation matrix
    double alpha = 90 * (M_PI / 180); // Estimated
    Mat R = (Mat_<double>(4, 4) <<
            1, 0, 0, 0,
            0, cos(-alpha), sin(-alpha), 0,
            0, -sin(-alpha), cos(-alpha), 0,
            0, 0, 0, 1);

    // Translation matrix
    double a = 5 * 1000; // Altitude in mm (estimated to be 10 m)
    Mat C = (Mat_<double>(3, 1) <<
            0,
            0,
            -a);

    // Helper 1
    Mat helper1;
    Mat eye1 = (Mat_<double>(1, 1) <<
            1);
    vconcat(-C, eye1, helper1);

    // Helper 2
    Mat helper2;
    Mat eye4 = (Mat_<double>(4, 3) <<
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            0, 0, 0);
    hconcat(eye4, helper1, helper2);

    // Camera matrix
    Mat P = K * R * helper2;

    // Simplification for plane
    Mat P_simplified = (Mat_<double>(3, 3) <<
            P.at<double>(0, 0), P.at<double>(0, 1), P.at<double>(0, 3),
            P.at<double>(1, 0), P.at<double>(1, 1), P.at<double>(1, 3),
            P.at<double>(2, 0), P.at<double>(2, 1), P.at<double>(2, 3));
    P = P_simplified;

    // Inverse camera matrix
    Mat PInverse;
    invert(P, PInverse);

    // Transformation matrix from {B} to {W}
    // TODO compute automatically
    Mat H_B_W = (Mat_<double>(3, 3) <<
            1 / 0.0152, 0, -126315.7894736843,
            0, 1 / 0.0152, 7813.852813852815,
            0, 0, 1);

    // C++ and MATLAB indexes from top left, but our coordinate system is from bottom right
    Mat H_I = (Mat_<double>(3, 3) <<
            -1, 0, w,
            0, -1, h,
            0, 0, 1);

    // Get {I} coordinates of point in {B} by first projecting from {B}
    // to {W} ({W} coordinates of the {B} point by rescaling
    // from pixels to mm) and then from {W} to {I}. Note that MATLAB and
    // OpenCV indexes images from top left corner, but our coordinate
    // system is from bottom right corner. So we have to convert b to
    // our coordinate system by H_I, then do the transformation, and
    // then convert back by H_I.
    Mat H_B_I = H_I * P * H_B_W * H_I;

    // Warp image
    warpPerspective(original_frame, original_frame, H_B_I, video_size, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
    warpPerspective(HSV_frame, HSV_frame, H_B_I, video_size, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);

    //    // This is manual application of transformation. It is much slower than OpenCV warpPerspective. The program becomes unresponsive. 
    //    
    //    // Initialize new image
    //    Mat B(original_frame.rows, original_frame.cols, original_frame.type());
    //    
    //    Mat b;
    //    Mat x;
    //
    //    // For each pixel of bird image {B}
    //    for (int b_y = 0; b_y < h; b_y++) {
    //        for (int b_x = 0; b_x < w; b_x++) {
    //
    //            // Point in {B}
    //            b = (Mat_<double>(3, 1) <<
    //                    b_x,
    //                    b_y,
    //                    1);
    //
    //            // Transform from {B} to {I}
    //            x = H_B_I * b;
    //            x = x / x.at<double>(2, 0);
    //
    //            // If the {I} is within the original image {I} boundaries
    //            if (0 <= x.at<double>(0, 0) && x.at<double>(0, 0) <= w && 0 <= x.at<double>(1, 0) && x.at<double>(1, 0) <= h) {
    //
    //                // Assign the bird point in {B} the same RGB value as corresponding
    //                // image point in {I}. 
    //                B.at<Vec3b>(b_y, b_x) = original_frame.at<Vec3b>(x.at<double>(1, 0), x.at<double>(0, 0));
    //
    //            }
    //
    //        }
    //    }
    //
    //    // Save new warped image into the original one
    //    original_frame = B;

    //        // Set value of particular pixel
    //    original_frame.at<Vec3b>(2027, 1876)[1] = 255;
    //    original_frame.at<Vec3b>(2027, 1876)[2] = 255;
    //    original_frame.at<Vec3b>(2027, 1876)[3] = 255;

    //        // Print type of original frame
    //    cout << original_frame.type() << endl;

    //    // Draw circle at the expected coordinates of EMILY in the first frame for 2016_05_10_lake_bryan.mov video and then directly above it at y = 0
    //    circle(original_frame, Point(2027, 1876), 5, Scalar(255, 255, 255));
    //    circle(original_frame, Point(2027, 0), 5, Scalar(255, 255, 255));

}