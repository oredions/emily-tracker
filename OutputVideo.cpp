/* 
 * File:   OutputVideo.cpp
 * Author: Jan Dufek
 */

#include "OutputVideo.hpp"
#include <iostream>

using namespace std;

OutputVideo::OutputVideo(double f, Size s, string n) {
    
    fps = f;
    size = s;
    name = n;
    
}

OutputVideo::OutputVideo(const OutputVideo& orig) {
}

OutputVideo::~OutputVideo() {
}

VideoWriter OutputVideo::get_video_writer() {
    
    // Codec used to output the video
    // This is higher size: int outputVideoCodec = CV_FOURCC('W','R','L','E');
    // This works navite on Mac: int outputVideoCodec = CV_FOURCC('m', 'p', '4', 'v');
    // This works with ffmpeg
    int output_video_codec = CV_FOURCC('D', 'I', 'V', 'X');

    // Open output video file
    VideoWriter output_video(name + ".avi", output_video_codec, fps, size, true);

    // Check if output video file was successfully opened
    if (!output_video.isOpened()) {
        cout << "Cannot open the output video file " << name + ".avi" << " for write." << endl;
    }
    
    return output_video;
}
