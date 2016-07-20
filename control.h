struct commands{
	float throttle; //0-1.0
	float rudder; //-1.0-1.0
};

commands get_control_commands(int xe, int ye, float theta, int xv, int yv);
