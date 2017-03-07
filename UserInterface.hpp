/* 
 * File:   UserInterface.hpp
 * Author: Jan Dufek
 */

#ifndef USERINTERFACE_HPP
#define USERINTERFACE_HPP

#include "opencv2/opencv.hpp"
#include "Settings.hpp"

using namespace std;
using namespace cv;

extern bool select_object;
extern int object_selected;
extern Rect selection;
extern Point target_location;
extern bool target_reached;
extern Point emily_pose_point_1;
extern Point emily_pose_point_2;

class UserInterface {
public:

    UserInterface(Settings&, Size);
    UserInterface(const UserInterface& orig);
    virtual ~UserInterface();
    
    void draw_position(int, int, double, Mat&);
    
    void draw_principal_axis(RotatedRect, Mat&);
    
    void draw_target(Mat&, Point);
    
    void print_status(Mat&, int, double);
    
    void drawAxis(Mat&, Point, Point, Scalar, const float);
    
    void show_main(Mat&);
    
    void show_histogram(Mat&);
    
private:
    
    void create_main_window();
    
    static void onMouse(int, int, int, int, void*);
    
    static void on_trackbar(int, void*);
    
    string int_to_string(int);
    
    // Program settings
    static Settings * settings;
    
    // Original point of click
    static Point origin;
    
    static Size video_size;

};

#endif /* USERINTERFACE_HPP */

