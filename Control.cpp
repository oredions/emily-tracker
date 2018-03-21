#include "Control.hpp"
#include <math.h> 
#include <stdio.h>
#include <iostream>

using namespace std;

#define PI 3.14159265

Control::Control() {
}

Control::Control(const Control& orig) {
}

Control::Control(Settings& s) {
    settings = &s;
}

Control::~Control() {
}

/**
 * Gets throttle limited by max_throttle determined by distance to target.
 * 
 * @param max_throttle maximum allowable throttle
 * @param distance_to_target distance to target
 * @return 
 */
double Control::get_throttle(double max_throttle, double distance_to_target) {

    // We are closer that slowing threshold
    if (distance_to_target < slowing_distance * settings->target_radius) {

        return max_throttle * (distance_to_target / ((double) slowing_distance * settings->target_radius));

    } else {

        return max_throttle;

    }
}

/**
 * Get the vector from USV to the target.
 * 
 * @param usv_x USV's X coordinate
 * @param usv_y USV's Y coordinate
 * @param target_x Target X coordinate
 * @param target_y Target Y coordinate
 * @param target_vector
 */
void Control::get_target_vector(double usv_x, double usv_y, double target_x, double target_y, double& target_vector){
    target_vector = atan2(target_y - usv_y, target_x - usv_x) * 180 / PI;
}

/**
 * Get the distance between the USV and the target.
 * 
 * @param usv_x USV's X coordinate
 * @param usv_y USV's Y coordinate
 * @param target_x Target X coordinate
 * @param target_y Target Y coordinate
 * @param distance_to_target
 */
void Control::get_distance_to_target(double usv_x, double usv_y, double target_x, double target_y, double& distance_to_target){
    distance_to_target = sqrt(pow(usv_x - target_x, 2) + pow(usv_y - target_y, 2));
}

/**
 * Get current control commands.
 * 
 * @param usv_x USV's X coordinate
 * @param usv_y USV's Y coordinate
 * @param theta USV's current heading
 * @param target_x Target X coordinate
 * @param target_y Target Y coordinate
 * @return 
 */
Command * Control::get_control_commands(double usv_x, double usv_y, double theta, double target_x, double target_y) {

    // PID proportional gain
    double kp = settings->proportional / 1000.0;

    // Get vector to target
    double target_vector;
    get_target_vector(usv_x, usv_y, target_x, target_y, target_vector);

    // Initialize current commands
    Command * current_commands = new Command(0, 0);

    // Get distance to target
    double distance_to_target;
    get_distance_to_target(usv_x, usv_y, target_x, target_y, distance_to_target);

    // Save distance to target
    current_commands->set_distance_to_target(distance_to_target);

    // Check if the target was reached
    if (distance_to_target < settings->target_radius) {
        
        // Already reached target
        cout << "Reached the target." << endl;
        target_reached = true;
        return current_commands;
        
    } else {
        
        // Target was not reached yet. Check the angle to target.
        
        // Angle is less than 180
        if (fabs(target_vector - theta) < 180) {

            // Save error angle to target
            current_commands->set_angle_error_to_target(target_vector - theta);

            // If the angle is less than given threshold, execute PID controller
            if (fabs(target_vector - theta) < 30) {

                current_commands->set_throttle(get_throttle(cruising_throttle, distance_to_target));
                current_commands->set_rudder(kp * (target_vector - theta));

            } else { // If the angle is more than given threshold, switch to turning mode

                current_commands->set_throttle(get_throttle(turning_throttle, distance_to_target));

                if (target_vector > theta) {
                    current_commands->set_rudder(1.0); //turn left is positive
                } else {
                    current_commands->set_rudder(-1.0); //turn right is negative
                }
                
            }
        }
        
        // Angle is more than 180, therefore consider turning in the other direction.
        if (fabs(target_vector - theta) >= 180) {
            
            // Compute angular difference
            if (target_vector > theta) {
                angle_difference = (target_vector - theta) - 360;
            } else {
                angle_difference = (target_vector - theta) + 360;
            }

            // Save error angle to target
            current_commands->set_angle_error_to_target(angle_difference);

            // If the angle is less than given threshold, execute PID controller
            if (fabs(angle_difference) < 30) {
                
                current_commands->set_throttle(get_throttle(cruising_throttle, distance_to_target));
                current_commands->set_rudder(kp * angle_difference);
                
            } else { // If the angle is more than given threshold, switch to turning mode
                
                current_commands->set_throttle(get_throttle(turning_throttle, distance_to_target));
                
                if (angle_difference > 0) {
                    current_commands->set_rudder(1.0); //turn left is positive
                } else {
                    current_commands->set_rudder(-1.0); //turn right is negative
                }
                
            }
        }
    }

    // Truncate rudder from above
    if (current_commands->get_rudder() > 1.0) {
        current_commands->set_rudder(1.0);
    }

    // Truncate rudder from bellow
    if (current_commands->get_rudder() < -1.0) {
        current_commands->set_rudder(-1.0);
    }

    return current_commands;
}