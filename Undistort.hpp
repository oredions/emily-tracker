/* 
 * File:   Undistort.hpp
 * Author: Jan Dufek
 */

#ifndef UNDISTORT_HPP
#define UNDISTORT_HPP

#include "opencv2/opencv.hpp"
#include "Settings.hpp"

#define PI 3.14159265

using namespace cv;

class Undistort {
public:
    Undistort(Settings&, Size);
    Undistort(const Undistort& orig);
    virtual ~Undistort();
    
    void undistort_camera(Mat&, Mat&);
    void undistort_perspective(Mat&, Mat&);
    
private:
    
    Settings * settings;
    Size video_size;
    
    Mat undistortRectifyMap1;
    Mat undistortRectifyMap2;

};

#endif /* UNDISTORT_HPP */

