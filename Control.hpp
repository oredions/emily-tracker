/* 
 * File:   Command.hpp
 * Author: Jan Dufek
 */

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "Command.hpp"
#include "Settings.hpp"

// Target was reached
extern bool target_reached;

class Control {
public:
    
    Control();
    Control(Settings&);
    Control(const Control& orig);
    virtual ~Control();

    Command * get_control_commands(int, int, double, int, int);

private:

    // Returns throttle based on distance to target
    double get_throttle(double, double);
    
    void get_distance_to_target(double xe, double ye, double xv, double yv, double& distance_to_target);
    
    void get_target_vector(double xe, double ye, double xv, double yv, double& target_vector);

    // Turning throttle
    const double turning_throttle = 0.3;

    // Cruising throttle
    const double cruising_throttle = 0.6;

    // Slowing distance as multiple of target radius. When this threshold is reached, EMILY will start linearly slowing down.
    int slowing_distance = 3;

    // Angle difference
    double angle_difference;
    
    // Program settings
    Settings * settings;

};

#endif /* CONTROL_HPP */