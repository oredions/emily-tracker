/* 
 * File:   OutputVideo.hpp
 * Author: Jan Dufek
 */

#ifndef OUTPUTVIDEO_HPP
#define OUTPUTVIDEO_HPP

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class OutputVideo {
public:
    OutputVideo(double, Size, string);
    OutputVideo(const OutputVideo& orig);
    virtual ~OutputVideo();
    
    VideoWriter get_video_writer();
    
private:
    double fps;
    Size size;
    string name;
};

#endif /* OUTPUTVIDEO_HPP */

