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

double Command::get_throttle() {
    return throttle;
}

void Command::set_throttle(double t) {
    throttle = t;
}

double Command::get_rudder() {
    return rudder;
}

void Command::set_rudder(double r) {
    rudder = r;
}

double Command::get_distance_to_target() {
    return distance_to_target;
}

void Command::set_distance_to_target(double d) {
    distance_to_target = d;
}

double Command::get_angle_error_to_target() {
    return angle_error_to_target;
}

void Command::set_angle_error_to_target(double a) {
    angle_error_to_target = a;
}