#include "control.h"
#include <math.h> 


#define PI 3.14159265

float turning_throttle = 0.3;
float cruising_throttle = 0.6;
float approach_radius = 5.0; //how many pixels away are defined as reach target
float kp = 0.1; //Proportional gain
float diff;

commands get_control_commands(int xe, int ye, float theta, int xv, int yv){
	float target_vector;
	target_vector = atan2(yv-ye,xv-xe)* 180 / PI;
	commands current_commands;
	current_commands. throttle = 0;
	current_commands.rudder = 0;
	float dist = sqrt((xe-xv)^2+(ye-yv)^2);
	if (dist<approach_radius) { // already reached target
		return current_commands;
	}
	else{
		if (fabs(target_vector-theta)<180) { //normal case
			if (fabs(target_vector-theta)<30){ //PID mode
				current_commands.throttle = cruising_throttle;
				current_commands.rudder = kp*(target_vector-theta);
			}
			else{ //turning mode
				current_commands.throttle = turning_throttle;
				if (target_vector>theta){
					current_commands.rudder = 1.0; //turn left is positive
				}
				else{
					current_commands.rudder = -1.0; //turn right is negative
				}
			}
		}
		if (fabs(target_vector-theta)>=180) {// consider turning in other direction
			if (target_vector>theta){
				diff = (target_vector-theta)-360;
			}
			else{
				diff = (target_vector-theta)+360;
			}
			if (fabs(diff)<30){ //PID mode
				current_commands.throttle = cruising_throttle;
				current_commands.rudder = kp*diff;
			}
			else{ //turning mode
				current_commands.throttle = turning_throttle;
				if (diff>0){
					current_commands.rudder = 1.0; //turn left is positive
				}
				else{
					current_commands.rudder = -1.0; //turn right is negative
				}
			}
		}
	}
	if (current_commands.rudder>1.0){
		current_commands.rudder=1.0;
	}
	if (current_commands.rudder<-1.0){
		current_commands.rudder=-1.0;
	}
	return current_commands;
	
}