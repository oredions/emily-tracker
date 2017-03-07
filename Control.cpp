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

void Control::get_target_vector(double xe, double ye, double xv, double yv, double& target_vector){
    target_vector = atan2(yv - ye, xv - xe) * 180 / PI;
}

void Control::get_distance_to_target(double xe, double ye, double xv, double yv, double& distance_to_target){
    distance_to_target = sqrt(pow(xe - xv, 2) + pow(ye - yv, 2));
}

Command * Control::get_control_commands(int xe_i, int ye_i, double theta, int xv_i, int yv_i) {

    double xe = (double) xe_i;
    double ye = (double) ye_i;
    double xv = (double) xv_i;
    double yv = (double) yv_i;

    double kp = settings->proportional / 1000.0;

    double target_vector;
    get_target_vector(xe, ye, xv, yv, target_vector);

    Command * current_commands = new Command(0, 0);

    double distance_to_target;
    get_distance_to_target(xe, ye, xv, yv, distance_to_target);

    // Save distance to target
    current_commands->set_distance_to_target(distance_to_target);

    if (distance_to_target < settings->target_radius) { // already reached target
        cout << "Reached the target." << endl;
        target_reached = true;
        return current_commands;
    } else {
        if (fabs(target_vector - theta) < 180) { //normal case

            // Save error angle to target
            current_commands->set_angle_error_to_target(target_vector - theta);

            if (fabs(target_vector - theta) < 30) { //PID mode

                current_commands->set_throttle(get_throttle(cruising_throttle, distance_to_target));
                current_commands->set_rudder(kp * (target_vector - theta));

            } else { //turning mode

                current_commands->set_throttle(get_throttle(turning_throttle, distance_to_target));

                if (target_vector > theta) {
                    current_commands->set_rudder(1.0); //turn left is positive
                } else {
                    current_commands->set_rudder(-1.0); //turn right is negative
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
            current_commands->set_angle_error_to_target(angle_difference);

            if (fabs(angle_difference) < 30) { //PID mode
                current_commands->set_throttle(get_throttle(cruising_throttle, distance_to_target));
                current_commands->set_rudder(kp * angle_difference);
            } else { //turning mode
                current_commands->set_throttle(get_throttle(turning_throttle, distance_to_target));
                if (angle_difference > 0) {
                    current_commands->set_rudder(1.0); //turn left is positive
                } else {
                    current_commands->set_rudder(-1.0); //turn right is negative
                }
            }
        }
    }

    if (current_commands->get_rudder() > 1.0) {
        current_commands->set_rudder(1.0);
    }

    if (current_commands->get_rudder() < -1.0) {
        current_commands->set_rudder(-1.0);
    }

    return current_commands;
}