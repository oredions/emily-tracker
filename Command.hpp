/* 
 * File:   Command.hpp
 * Author: Jan Dufek
 */

#ifndef COMMAND_HPP
#define COMMAND_HPP

class Command {
public:
    Command();
    Command(const Command& orig);
    Command(double, double);
    virtual ~Command();
    
    double get_throttle();
    void set_throttle(double);
    
    double get_rudder();
    void set_rudder(double);
    
    double get_distance_to_target();
    void set_distance_to_target(double);
    
    double get_angle_error_to_target();
    void set_angle_error_to_target(double);
    
private:
    double throttle;
    double rudder;
    double distance_to_target;
    double angle_error_to_target;
};

#endif /* COMMAND_HPP */

