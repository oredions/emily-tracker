#include "control.h"
#include <math.h> 
#include <stdio.h>
#include <iostream>

using namespace std;

#define PI 3.14159265

const double turning_throttle = 0.3;
const double cruising_throttle = 0.6;
//const double approach_radius = 100.0; //how many pixels away are defined as reach target // TODO put in gui
//const double kp = 0.01; //Proportional gain
double diff;

commands get_control_commands(int xe_i, int ye_i, double theta, int xv_i, int yv_i) {

    double xe = (double) xe_i;
    double ye = (double) ye_i;
    double xv = (double) xv_i;
    double yv = (double) yv_i;
    
    double kp = proportional / 1000.0;

    double target_vector;
    target_vector = atan2(yv - ye, xv - xe)* 180 / PI;
    commands current_commands;
    current_commands.throttle = 0;
    current_commands.rudder = 0;
    double dist = sqrt(pow(xe - xv, 2) + pow(ye - yv, 2));

    if (dist < target_radius) { // already reached target
        cout << "Reached the target." << endl;
        target_reached = true;
        return current_commands;
    } else {
        if (fabs(target_vector - theta) < 180) { //normal case
            if (fabs(target_vector - theta) < 30) { //PID mode
                current_commands.throttle = cruising_throttle;
                current_commands.rudder = kp * (target_vector - theta);
            } else { //turning mode
                current_commands.throttle = turning_throttle;
                if (target_vector > theta) {
                    current_commands.rudder = 1.0; //turn left is positive
                } else {
                    current_commands.rudder = -1.0; //turn right is negative
                }
            }
        }
        if (fabs(target_vector - theta) >= 180) {// consider turning in other direction
            if (target_vector > theta) {
                diff = (target_vector - theta) - 360;
            } else {
                diff = (target_vector - theta) + 360;
            }
            if (fabs(diff) < 30) { //PID mode
                current_commands.throttle = cruising_throttle;
                current_commands.rudder = kp * diff;
            } else { //turning mode
                current_commands.throttle = turning_throttle;
                if (diff > 0) {
                    current_commands.rudder = 1.0; //turn left is positive
                } else {
                    current_commands.rudder = -1.0; //turn right is negative
                }
            }
        }
    }
    if (current_commands.rudder > 1.0) {
        current_commands.rudder = 1.0;
    }
    if (current_commands.rudder<-1.0) {
        current_commands.rudder = -1.0;
    }
    return current_commands;
}