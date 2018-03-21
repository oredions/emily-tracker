/* 
 * File:   Command.cpp
 * Author: Jan Dufek
 */

#include "Command.hpp"

Command::Command() {
}

Command::Command(const Command& orig) {
}

Command::Command(double t, double r) {
    throttle = t;
    rudder = t;
}

Command::~Command() {
}

/**
 * Get throttle value.
 * 
 * @return 
 */
double Command::get_throttle() {
    return throttle;
}

/**
 * Set throttle value.
 * 
 */
void Command::set_throttle(double t) {
    throttle = t;
}

/**
 * Get rudder value.
 * 
 * @return 
 */
double Command::get_rudder() {
    return rudder;
}

/**
 * Set rudder value.
 * 
 */
void Command::set_rudder(double r) {
    rudder = r;
}

/**
 * Get distance between the USV and the target.
 * 
 * @return 
 */
double Command::get_distance_to_target() {
    return distance_to_target;
}

/**
 * Set distance between the USV and the target.
 * 
 */
void Command::set_distance_to_target(double d) {
    distance_to_target = d;
}

/**
 * Get angle between the USV and the target.
 * 
 * @return 
 */
double Command::get_angle_error_to_target() {
    return angle_error_to_target;
}

/**
 * Set angle between the USV and the target.
 * 
 */
void Command::set_angle_error_to_target(double a) {
    angle_error_to_target = a;
}