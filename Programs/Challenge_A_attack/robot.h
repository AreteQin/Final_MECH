
// mobile robot simulation class

const int NS_MAX = 20; // maximum number of state variables

class robot {
	
	public:
	
	int N; // number of state variables
	
	double t; // current time (seconds)
	double x[NS_MAX+1];  // state vector x
	double xd[NS_MAX+1]; // derivative vector at time t
	
	double vl, vr; // left and right wheel velocity magnitude / speed
	// (velocity direction is assumed perpedicular to shaft)
		
	// reference / desired value of alpha
	double alpha_ref;
		
	// maximum wheel speed (pixels/s)
	double v_max;
		
	double D; // distance between wheels (ie shaft length) (pixels)
	
	// position of laser in local robot coordinates
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	double Lx, Ly;
	
	// position of robot axis of rotation (half way between wheels)
	// and the robot image center (pixels) in local coord
	double Ax, Ay;
	
	// max range of laser / gripper (rad)
	double alpha_max;
	
	// position of gripper center (outputs) (pixels) in global coord
	double xg, yg; 
	
	// position of robot axis with respect in global coord (pixels)
	double xa, ya; 
	
	// robot laser state (0 - off, 1 - on)
	int laser;
	
	robot(double x0, double y0, double theta0, double vmax);

	void sim_step(double dt);

	void set_inputs(int pw_l, int pw_r, int pw_laser, int laser);

	void calculate_outputs();
	
};

