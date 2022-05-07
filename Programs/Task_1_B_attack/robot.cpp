
// mobile robot simulation class

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

#include "robot.h"

using namespace std;

robot::robot(double x0, double y0, double theta0, double vmax)
{
	// model parameters
	
	N = 4; // number of state variables
	
	// note units for D, Lx, Ly are pixels
	D = 121.0;
	
	// position of laser in local robot coordinates
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction	
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation (half way between wheels)
	// and the robot image center (pixels)
	Ax = 37.0;
	Ay = 0.0;
	
	// max range of laser / gripper (rad)
	alpha_max = 3.14159/2;	
	
	// note units vmax is pixels/s
	v_max = vmax;	

	// initial conditions
	
	t = 0.0; // initial time (s)

	x[1] = theta0; // rad/s
	x[2] = x0; // pixels
	x[3] = y0; // pixels
	x[4] = 0.0; // rad
	
	// set initial inputs
	vl = 0.0; // pixels/s
	vr = 0.0; // pixels/s
	
	alpha_ref = 0.0; // rad
	
	// robot laser state (0 - off, 1 - on)
	laser = 0;
	
	// calculate the gripper position / output
	calculate_outputs();
	
}


void robot::sim_step(double dt)
{
	int i;
	double v; // forward velocity of the robot
	double w; // angular velocity of robot	
	double alpha;
	double EPS;
		
	// maximum speed (rad/s) for laser servo
	double alpha_dot_max = 3.5;
		
	// * assume inputs vl and vr have already been set
	// using set_inputs, etc.
	
	// calculate the derivative vector at time t /////////////
		
	// forward velocity of robot (dir perpendicular to shaft)
	v = (vl + vr) / 2;

	// angular velocity of robot
	w = (vr - vl) / D;
		
	// robot state variable equations
		
	xd[1] = w; // theta_dot		
	xd[2] = v*cos(x[1]); // x_dot
	xd[3] = v*sin(x[1]); // y_dot

	alpha = x[4];
	if( alpha < alpha_ref ) xd[4] = alpha_dot_max;
	if( alpha > alpha_ref ) xd[4] = -alpha_dot_max;

	// stop when close to alpha_ref
	EPS = 0.0017;
	if( abs(alpha - alpha_ref) < EPS ) {
		xd[4] = 0.0;
		x[4]  = alpha_ref;
	}
	
	///////////////////////////////////////////////////////////

	// use Euler's method to calculate x(t+dt)
	for(i=1;i<=N;i++) x[i] = x[i] + xd[i]*dt;

	t = t + dt; // increment time

	// calculate outputs
	
	// calculate the gripper position / output
	calculate_outputs();

}


void robot::set_inputs(int pw_l, int pw_r, int pw_laser, int laser)
{	
	// set laser input
	this->laser = laser;
	
	// neutral pulse width (us)
	// assume wheels stop at this pulse width value
	int pw_0 = 1500; 
	
	// maximum range of pulse width -- 1500 +/- 500 range
	int pw_range = 500; 
	
	// input limits / saturation values
	int pw_min = pw_0 - pw_range;
	int pw_max = pw_0 + pw_range;
	
	// enforce / saturate input limits
	
	if( pw_l < pw_min ) pw_l = pw_min;
	if( pw_l > pw_max ) pw_l = pw_max;	
	
	if( pw_r < pw_min ) pw_r = pw_min;	
	if( pw_r > pw_max ) pw_r = pw_max;
	
	if( pw_laser < pw_min ) pw_laser = pw_min;	
	if( pw_laser > pw_max ) pw_laser = pw_max;	
	
	// assume wheel velocities are in opposite directions since
	// the servos are flipped on the robot
	
	vl = ( (double)pw_l - pw_0 ) / pw_range; // -1 to 1 range
	vl *= -v_max; // - sign due to flipped servos
	
	vr = ( (double)pw_r - pw_0 ) / pw_range; // -1 to 1 range
	vr *= v_max;
	
	alpha_ref = ( (double)pw_laser - pw_0 ) / pw_range; // -1 to 1 range
	alpha_ref *= alpha_max;
	
}


void robot::calculate_outputs()
{
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction
	
	// pg = T + R*pl,  pg = [xg,yg], pl = [Lx,Ly]	
	// R  = [cos(th) -sin(th)]
	//      [sin(th)  cos(th)]
	
	// calculate gripper / laser position / output in global coord (pixels)
	xg = x[2] + Lx*cos(x[1]) - Ly*sin(x[1]);
	yg = x[3] + Lx*sin(x[1]) + Ly*cos(x[1]);
	
	// calculate robot center of rotation in global coord (pixels)
	xa = Ax*cos(x[1]) - Ay*sin(x[1]);
	ya = Ax*sin(x[1]) + Ay*cos(x[1]);
	
}
	