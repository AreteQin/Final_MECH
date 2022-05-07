
const int N_MAX = 100;
	
class robot_system {
	public:

	// robot system simulation time
	double t;

	// number of robots in the simulation
	int N_robot;

	// array of pointers to robot objects
	robot *P[N_MAX];

	// simulated image width and height
	double width, height;

	// obstacle locations and sizes
	int N_obs;
	double x_obs[N_MAX];
	double y_obs[N_MAX];
	double size_obs[N_MAX];

	// lighting condition parameters (TODO: add shadow effect, etc.)
	double light;
	double light_gradient;
	double light_dir;
	double image_noise;

	// 1 player, 2 player, practice, strategies, speed / difficulty level
	int mode;
	int level;

	// constructor -- set default simulation parameters
	robot_system(double D, double Lx, double Ly, 
		double Ax, double Ay, double alpha_max, int n_robot);
	
	~robot_system();
	
	void sim_step(double dt);
};


int activate_simulation(double width, double height,
	double x_obs[], double y_obs[], double size_obs[], int N_obs,
	char robot_file[], char opponent_file[], char background_file[],
	char obstacle_file[], double D, double Lx, double Ly, 
	double Ax, double Ay, double alpha_max, int n_robot);
	
int deactivate_simulation();

int set_inputs(int pw_l, int pw_r, int pw_laser, int laser, 
	double light, double light_gradient, double light_dir,
	double image_noise, double max_speed, double opponent_max_speed);
	
int set_opponent_inputs(int pw_l, int pw_r, int pw_laser, int laser, 
				double max_speed);	

int acquire_image_sim(image &rgb);

int set_simulation_mode(int mode, int level);

int set_robot_position(double x, double y, double theta);

int set_opponent_position(double x, double y, double theta);

int rotate(image &a, image &b, image &c, double theta, int ig, int jg);
	
int append(image &a, image &b, int ip, int jp);

int draw_laser(robot *P, image &rgb);

int draw_point_rgb_laser(image &rgb, int ip, int jp, int R, int G, int B);

int wait_for_player();

int join_player();
