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