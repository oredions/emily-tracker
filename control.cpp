#include "control.h"
#include "settings.h"
#include <math.h> 
#include <stdio.h>
#include <iostream>

using namespace std;

#define PI 3.14159265

// Turning throttle
const double turning_throttle = 0.3;

// Cruising throttle
const double cruising_throttle = 0.6;

// Angle difference
double angle_difference;

/**
 * Gets throttle limited by max_throttle determined by distance to target.
 * 
 * @param max_throttle maximum allowable throttle
 * @param distance_to_target distance to target
 * @return 
 */
double get_throttle(double max_throttle, double distance_to_target) {
    
    // We are closer that slowing threshold
    if (distance_to_target < slowing_distance * target_radius) {
        
        return max_throttle * (distance_to_target / ((double) slowing_distance * target_radius));
        
    } else {
        
        return max_throttle;
        
    }
}

commands get_control_commands(int xe_i, int ye_i, double theta, int xv_i, int yv_i) {

    double xe = (double) xe_i;
    double ye = (double) ye_i;
    double xv = (double) xv_i;
    double yv = (double) yv_i;

    double kp = proportional / 1000.0;

    double target_vector;
    target_vector = atan2(yv - ye, xv - xe) * 180 / PI;
    
    commands current_commands;
    current_commands.throttle = 0;
    current_commands.rudder = 0;
    
    double distance_to_target = sqrt(pow(xe - xv, 2) + pow(ye - yv, 2));

    // Save distance to target
    current_commands.distance_to_target = distance_to_target;

    if (distance_to_target < target_radius) { // already reached target
        cout << "Reached the target." << endl;
        target_reached = true;
        return current_commands;
    } else {
        if (fabs(target_vector - theta) < 180) { //normal case

            // Save error angle to target
            current_commands.angle_error_to_target = target_vector - theta;

            if (fabs(target_vector - theta) < 30) { //PID mode

                current_commands.throttle = get_throttle(cruising_throttle, distance_to_target);
                current_commands.rudder = kp * (target_vector - theta);
                
            } else { //turning mode
                
                current_commands.throttle = get_throttle(turning_throttle, distance_to_target);
                
                if (target_vector > theta) {
                    current_commands.rudder = 1.0; //turn left is positive
                } else {
                    current_commands.rudder = -1.0; //turn right is negative
                }
            }
        }
        if (fabs(target_vector - theta) >= 180) {// consider turning in other direction
            if (target_vector > theta) {
                angle_difference = (target_vector - theta) - 360;
            } else {
                angle_difference = (target_vector - theta) + 360;
            }

            // Save error angle to target
            current_commands.angle_error_to_target = angle_difference;

            if (fabs(angle_difference) < 30) { //PID mode
                current_commands.throttle = get_throttle(cruising_throttle, distance_to_target);
                current_commands.rudder = kp * angle_difference;
            } else { //turning mode
                current_commands.throttle = get_throttle(turning_throttle, distance_to_target);
                if (angle_difference > 0) {
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

    if (current_commands.rudder < -1.0) {
        current_commands.rudder = -1.0;
    }

    return current_commands;
}