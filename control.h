struct commands{
	double throttle; //0-1.0
	double rudder; //-1.0-1.0
};

// Target was reached
extern bool target_reached;
extern int target_radius;
extern int proportional;

commands get_control_commands(int xe, int ye, double theta, int xv, int yv);
