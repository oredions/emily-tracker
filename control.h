struct commands{
	double throttle; //0-1.0
	double rudder; //-1.0-1.0
        double distance_to_target;
        double angle_error_to_target;
};

// Target was reached
extern bool target_reached;
extern int target_radius;
extern int proportional;

commands get_control_commands(int xe, int ye, double theta, int xv, int yv);
