
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

#include "shared_memory.h"

// global variables ///////////

robot_system *S1;
image rgb_robot, rgb_opponent, rgb_background;
image rgb_robot_r, rgb_opponent_r;
image rgb_obstacle, rgb_obstacle_r;

char *p_shared; // pointer to shared memory

///////////////////////////////

int activate_simulation(double width, double height,
	double x_obs[], double y_obs[], double size_obs[], int N_obs,
	char robot_file[], char opponent_file[], char background_file[],
	char obstacle_file[], double D, double Lx, double Ly, 
	double Ax, double Ay, double alpha_max, int n_robot)
{
	int i, k;
	double *pd;	
	int *pi;

	S1 = new robot_system(D,Lx,Ly,Ax,Ay,alpha_max,n_robot);

	if( S1 == NULL ) {
		cout << "\nmemory allocation error in activate_simulation()";
		return 1;
	}
	
	S1->width = width;
	S1->height = height;

	S1->N_obs = N_obs;

	for(i=1;i<=N_obs;i++) {
		S1->x_obs[i] = x_obs[i];
		S1->y_obs[i] = y_obs[i];
		S1->size_obs[i] = size_obs[i];
	}

	// get image size, dynamically allocate images, and load from file
	
	set_rgb_image(robot_file,rgb_robot);
	set_rgb_image(robot_file,rgb_robot_r);	
		
	set_rgb_image(opponent_file,rgb_opponent);
	set_rgb_image(opponent_file,rgb_opponent_r);
	
	set_rgb_image(background_file,rgb_background);

	set_rgb_image(obstacle_file,rgb_obstacle);
	set_rgb_image(obstacle_file,rgb_obstacle_r);
		
	// for testing ///////////////////

/*	
	rotate(rgb_robot,rgb_robot_r,rgb_background,3.14159/60,230,170);	
	append(rgb_background,rgb_robot_r,230,170);

//	save_rgb_image("output7.bmp",rgb_robot_r);
	save_rgb_image("output7.bmp",rgb_background);

	cout << "\noutput file complete.\n";
	Sleep(1000);
	exit(0);
*/	

	// setup shared memory for 2 player option ///////////
	
	int n_shared = 1000; // size of shared memory block (bytes)
	char name[] = "shared_memory_v"; // name of shared memory block
	
	// create / access shared memory
	p_shared = shared_memory(name,n_shared);
	
	// player 1
	k = 0;
	pi = (int *)(p_shared + k);
	*pi = 0; pi++; // sample
	*pi = 0; pi++; // laser
	pd = (double *)pi;
	*pd = 0.0; pd++; // theta
	*pd = 150; pd++; // x
	*pd = 150; pd++; // y
	*pd = 0.0; pd++; // alpha

	// player 2
	k = 500;
	pi = (int *)(p_shared + k);
	*pi = 0; pi++; // sample
	*pi = 0; pi++; // laser
	pd = (double *)pi;
	*pd = 0.0; pd++; // theta
	*pd = 300; pd++; // x
	*pd = 250; pd++; // y
	*pd = 0.0; pd++; // alpha

	return 0;
}


int wait_for_player()
{
	int k, *pi;	
	
	k = 900;
	pi = (int *)(p_shared + k);
	
	// initialize start flag
	*pi = 0;
	
	cout << "\n\nwaiting for player to join ...\n";
	
	// wait for player to join
	while(1) {
		if( *pi == 1 ) break;
		Sleep(1);
	}
	
	return 0;
}


int join_player()
{
	int k, *pi;	
	
	k = 900;
	pi = (int *)(p_shared + k);
	
	cout << "\n\njoining other player ...\n";	
	
	// indicate player is ready to join
	*pi = 1;
	
	return 0;
}


int deactivate_simulation()
{
	// free the image memory before the program completes
	free_image(rgb_robot);
	free_image(rgb_opponent);
	free_image(rgb_background);
	free_image(rgb_robot_r);
	free_image(rgb_opponent_r);
	free_image(rgb_obstacle);
	free_image(rgb_obstacle_r);	
	
	// safe delete
	if( S1 != NULL ) {
		delete S1;
		S1 = NULL;
	} else {
		cout << "\nerror: NULL pointer in deactivate_simulation()";
	}
	
	return 0;
}

// TODO: interpolate inputs if large delays ?

int set_inputs(int pw_l, int pw_r, int pw_laser, int laser, 
	double light, double light_gradient, double light_dir,
	double image_noise, double max_speed, double opponent_max_speed)
{
	// * note it's the responsibility of the user to call this function
	// or the inputs will stay constant in the simulation
	
	// set inputs for the simulation -- no smoothing / interpolation
	// since actual system doesn't do that
	
	S1->light = light;
	S1->light_gradient = light_gradient;
	S1->light_dir = light_dir;
	S1->image_noise = image_noise;

	// need to set max_speed before setting other inputs
	// since inputs depend on that parameter
	S1->P[1]->v_max = max_speed;	

	// set robot inputs
	S1->P[1]->set_inputs(pw_l,pw_r,pw_laser,laser);
	
	if( S1->N_robot > 1 ) {
		S1->P[2]->v_max = opponent_max_speed;
	}
	
	// note the opponent input will be set automatically later
	
	return 0;
}


int set_opponent_inputs(int pw_l, int pw_r, int pw_laser, int laser, 
				double max_speed)
{
	// manually set opponent inputs for the simulation
	// -- good for testing your program

	if( S1->N_robot > 1 ) {
		
		// set opponent parameters
		S1->P[2]->v_max = max_speed;	

		// set opponent inputs
		S1->P[2]->set_inputs(pw_l,pw_r,pw_laser,laser);
		
	}
	
	return 0;
}


// TODO:
// 1. precompute -- in activate to save time
// eg image rotations, etc.

// 2. draw laser gun turrent and laser blast
// precomputed laser blasts at different angles 
// use bmp and rotate for lasers -- how to shorten ?
// -- just draw multiple rgb points ?


int acquire_image_sim(image &rgb)
// assume this function is called frequenlty since it performs
// real-time simulation of the robots
{
	int i, j, k;
	double x, y, theta;
	int ic, jc;
	
	static int init = 0;
	static double tc0;
	double tc;
	double dt = 1.0e-4; // simulation time step
	
	// laser start times
	static double t_laser_start = 0.0, t_laser_start_o = 0.0;	
	
	// previous state of laser inputs
	static int laser_previous = 0, laser_previous_o = 0;
	static int laser_fired = 0, laser_fired_o = 0;

	// laser time durations (s)
	double t_laser, t_laser_o;
	double laser_duration = 1.0;
	int laser_duration_ms = (int)(laser_duration*1000);
	
	double theta_s, x_s, y_s, alpha_s, *pd;	
	int laser_s, sample_s = 0, *pi;

	// TODO: use subpixel rendering -- ie double for (i,j), etc.
	
	// TODO: time stamp result with simulation and or clock time

	// store initial clock time
	if( !init ) {
		tc0 = high_resolution_time();
		init = 1;
	}

	// read current clock time
	tc = high_resolution_time() - tc0;

	// freeze simulation if either robot or opponent laser is fired
	if( laser_fired ) {
		Sleep(laser_duration_ms);
		// adjust tc0 so the real-time simulation stops
		tc0 = high_resolution_time() - tc;
		laser_fired = 0; // turn off laser when done
	}
	
	if( laser_fired_o ) {
		Sleep(laser_duration_ms);
		// adjust tc0 so the real-time simulation stops
		tc0 = high_resolution_time() - tc;
		laser_fired_o = 0; // turn off laser when done
	}	

	// real-time simulation of robots
	// -- simulate robots with time step dt until simulation time = clock time
	while( S1->t < tc ) S1->sim_step(dt);

	// read/write shared memory to get/set the state of the 
	// opponent/robot for 2 player mode //////////////////////

	// mode = 1 - two player mode, player #1 (data block #1)
	// mode = 2 - two player mode, player #2 (data block #2)
	
	if( (S1->mode == 1) || (S1->mode == 2) ) {
		
		// read opponent state from shared memory block ////
		
		if( S1->mode == 1 ) {
			k = 500; // byte number for player 2 data block
		} else {
			k = 0; // byte number for player 1 data block
		}
		
		// start of memory block to write robot data	
		pi = (int *)(p_shared + k);
		
		sample_s = *pi; pi++; // sample
		laser_s = *pi; pi++; // laser
		pd = (double *)pi;
		theta_s = *pd; pd++; // theta
		x_s = *pd; pd++; // x
		y_s = *pd; pd++; // y
		alpha_s = *pd; pd++; // alpha			
				
		// set opponent states to read data
		S1->P[2]->x[1] = theta_s;	
		S1->P[2]->x[2] = x_s;
		S1->P[2]->x[3] = y_s;
		S1->P[2]->x[4] = alpha_s;	
		S1->P[2]->laser = laser_s;
				
		// write robot state to shared memory block ////////
		
		if( S1->mode == 1 ) {
			k = 0; // byte number for player 1 data block
		} else {
			k = 500; // byte number for player 2 data block
		}	
		
		// start of memory block to write robot data	
		pi = (int *)(p_shared + k);		
		
		// get robot data to write
		theta_s = S1->P[1]->x[1];	
		x_s     = S1->P[1]->x[2];
		y_s     = S1->P[1]->x[3];
		alpha_s = S1->P[1]->x[4];	
		laser_s = S1->P[1]->laser;	
		
		*pi = sample_s; pi++; // sample
		*pi = laser_s; pi++; // laser
		pd = (double *)pi;
		*pd = theta_s; pd++; // theta
		*pd = x_s; pd++; // x
		*pd = y_s; pd++; // y
		*pd = alpha_s; pd++; // alpha	
		
	}
	
	// construct image from simulation results /////////
	
	// copy background into result
	copy(rgb_background,rgb);

	// add obstacles to image /////////////////
	
	// obstacle image center point
	ic = rgb_obstacle.width / 2;
	jc = rgb_obstacle.height / 2;	
	
	for(k=1;k<=S1->N_obs;k++) {
		// get pixel location of obstacle center
		i = (int)S1->x_obs[k];
		j = (int)S1->y_obs[k];
		
		// calculate bottom left corner of image in global coord
		i -= ic;
		j -= jc;		
		
		// place obstacle at point (i,j)
		append(rgb,rgb_obstacle,i,j);
		
//		S1->size_obs[i];
	}

	// add opponent to image ///////////////////
	
	if( S1->N_robot > 1 ) {
		
		theta = S1->P[2]->x[1] - 3.14159/2;
		x 	  = S1->P[2]->x[2] - S1->P[2]->xa;
		y 	  = S1->P[2]->x[3] - S1->P[2]->ya;
	
		// opponent image center point
		ic = rgb_opponent.width / 2;
		jc = rgb_opponent.height / 2;
	
		// get pixel location of opponent center
		i = (int)x;
		j = (int)y;	
		
		// calculate bottom left corner of image in global coord
		i -= ic;
		j -= jc;

		// rotate opponent image around image center point by theta	
		rotate(rgb_opponent,rgb_opponent_r,rgb_background,theta,i,j);

		// place opponent at point (i,j)
		append(rgb,rgb_opponent_r,i,j);
		
	}
	
	// add robot to image //////////////////////
	
	theta = S1->P[1]->x[1] - 3.14159/2;	
	x 	  = S1->P[1]->x[2] - S1->P[1]->xa;
	y 	  = S1->P[1]->x[3] - S1->P[1]->ya;
	
	// robot image center point
	ic = rgb_robot.width / 2;
	jc = rgb_robot.height / 2;
	
	// get pixel location of robot center
	i = (int)x;
	j = (int)y;		
		
	// calculate bottom left corner of image in global coord
	i -= ic;
	j -= jc;

//	theta = 3.14159/4; // for testing

	// rotate robot image around image center point by theta	
	rotate(rgb_robot,rgb_robot_r,rgb_background,theta,i,j);

	// place robot at point (i,j)
	append(rgb,rgb_robot_r,i,j);

	// for testing /////////////
/*
	save_rgb_image("output7.bmp",rgb);

	cout << "\noutput file complete.\n";
	Sleep(1000);
	exit(0);
*/	

	// draw laser when fired /////////////////////////

	// check if laser was fired (laser went from 0 to 1)
	// -- ie rising edge on laser
	if(	S1->P[1]->laser && !laser_previous ) {
		t_laser_start = S1->t; // record laser start time
		laser_fired = 1;
		cout << "\nlaser fired !";
	}
	
	// update previous laser state
	laser_previous = S1->P[1]->laser;
	
	// draw laser if fired
	if( laser_fired ) {
		draw_laser(S1->P[1],rgb);
	}
	
/*
	// this code is for case where laser doesn't freeze simulation
	// turn laser off after duration
	t_laser = S1->t - t_laser_start;
	if( t_laser > laser_duration ) {
		laser_fired = 0;
	}
*/
	
	// fire opponent laser if needed
	if( S1->N_robot > 1 ) {
		
		// check if laser was fired (laser went from 0 to 1)
		// -- ie rising edge on laser
		if(	S1->P[2]->laser && !laser_previous_o ) {
			t_laser_start_o = S1->t; // record laser start time
			laser_fired_o = 1;
			cout << "\nopponent laser fired !";
		}
	
		// update previous laser state
		laser_previous_o = S1->P[2]->laser;
	
		// draw laser if fired
		if( laser_fired_o ) {
			draw_laser(S1->P[2],rgb);
		}
				
	}
	
	return 0;
}


int rotate(image &a, image &b, image &c, double theta, int ig, int jg)
{
	int i, j, k, R, G, B;
	int width, height;
	ibyte *pa, *pb, *pc;
	double i_l, j_l; // local coord for i,j
	double ic, jc;
	double cos_th, sin_th;
	int i1, j1, i2, j2;
	int sum, width_c, height_c;
	
	// variables for bilinear interpolation
	ibyte *p1, *p2, *p3, *p4;
	int B1, G1, R1, B2, G2, R2, B3, G3, R3, B4, G4, R4;	
	double Ba, Ga, Ra, Bb, Gb, Rb, ip, jp;
	int flag;

	// TODO: add #define DEBUG which checks bounds

	// TODO: check for image compatibility

	cos_th = cos(theta);
	sin_th = sin(theta);
	
	pa = a.pdata;
	pb = b.pdata;	
	pc = c.pdata;
	
	// width of object images a and b
	width = b.width;
	height = b.height;
	
	// width of background image c
	width_c = c.width;
	height_c = c.height;	
	
	// erase output image
	R = 0; G = 0; B = 0;
	for(j=0;j<height;j++) {		
		for(i=0;i<width;i++) {
			*pb		= B;
			*(pb+1) = G;
			*(pb+2) = R;
			pb += 3; // move to next pixel in output image
		}
	}
	
	// initialize pb for next step
	pb = b.pdata;	
	
	// calculate each pixel in output image by linear interpolation
	// with respect to original image
	for(j=0;j<height;j++) {		
	
		for(i=0;i<width;i++) {
			
			// calculate local coord il, jl
			// pg = R*pl,  pg = [i,j], pl = [il,jl]	
			// pl = inv(R)*pg = Rt*pg
			// R  = [cos(th) -sin(th)]
			//      [sin(th)  cos(th)]
			// Rt = [ cos(th) sin(th)]
			//      [-sin(th) cos(th)]
			
			// center coords
			ic = i - 0.5*width;
			jc = j - 0.5*height;
			
			i_l =  cos_th*ic + sin_th*jc;
			j_l = -sin_th*ic + cos_th*jc;
			
			i1 = (int)(i_l + 0.5*width);
			i2 = i1 + 1;
			
			j1 = (int)(j_l + 0.5*height);
			j2 = j1 + 1;			
			
			// check if (i1,j1), etc. are within range
			if( (i1 > 3) && (i1 < width-3) && 
				(j1 > 3) && (j1 < height-3) ) {
					
/*		
			// simple interpolation -- use (i1,j1) ///////////
				
			// byte k for pixel (i1,j1)
			k = 3*( j1*width + i1 );
			
			B = pa[k];
			G = pa[k+1];
			R = pa[k+2];
			
			////////////////////////////////////////////////
*/			

			// bilinear interpolation ///////////////////////

			// set neighborhood pointers to interpolation points
			// p3 p4
			// p1 p2
			
			// TODO: remove i,j calcs and use small increments like conv
			p1 = pa + 3*( j1*width + i1 ); // (i1,j1)			
			p2 = pa + 3*( j1*width + i2 ); // (i2,j1)
			p3 = pa + 3*( j2*width + i1 ); // (i1,j2)
			p4 = pa + 3*( j2*width + i2 ); // (i2,j2)
			
			B1 = *p1; G1 = *(p1+1); R1 = *(p1+2);
			B2 = *p2; G2 = *(p2+1); R2 = *(p2+2);	
			B3 = *p3; G3 = *(p3+1); R3 = *(p3+2);
			B4 = *p4; G4 = *(p4+1); R4 = *(p4+2);				
			
			sum = B1 + G1 + R1 + B2 + G2 + R2 + B3 + G3 + R3 + B4 + G4 + R4;		
			
			// interpolate if at least one interpolation point is not zero
			if( sum > 0 ) {
			
			// replace black interpolation points with background points		
			
			// TODO: combine rotate with append functions
			// for efficiency and also subpixel resolution
			// with bilinear interpolation ?
			// what happens if ig, jg is subpixel ?
			
			// check for background pixel out of range
			// and use nearest background pixel
			if( i + ig < 0 ) ig = -i;
			if( i + ig >= width_c ) ig = width_c - i - 1;
			if( j + jg < 0 ) jg = -j;
			if( j + jg >= height_c ) jg = height_c - j - 1;

			flag = 0;
			if ( B1 + G1 + R1 == 0 ) {
				p1 = pc + 3*( (j + jg) * width_c + i + ig );		
				B1 = *p1; G1 = *(p1+1); R1 = *(p1+2);
				flag = 1;
			}
			
			if ( B2 + G2 + R2 == 0 ) {
				p2 = pc + 3*( (j + jg) * width_c + i + ig );	
				B2 = *p2; G2 = *(p2+1); R2 = *(p2+2);	
				flag = 1;				
			}

			if ( B3 + G3 + R3 == 0 ) {
				p3 = pc + 3*( (j + jg) * width_c + i + ig );		
				B3 = *p3; G3 = *(p3+1); R3 = *(p3+2);
				flag = 1;
			}

			if ( B4 + G4 + R4 == 0 ) {
				p4 = pc + 3*( (j + jg) * width_c + i + ig );
				B4 = *p4; G4 = *(p4+1); R4 = *(p4+2);	
				flag = 1;			
			}			

/*			
			if( flag ) {
				p1 = pc + 3*( (j + jg) * width_c + i + ig + 1);	
				*p1 = 0;
				*(p1+1) = 255;
				*(p1+2) = 0;			
			}
*/			
			/////////////////////////////////////////////////////
			
			// find RGB for point a, b using linear interpolation, eg
			// p3 Rb p4
			// p1 Ra p2
	
			// interpolation point
			ip = i_l + 0.5*width;
			jp = j_l + 0.5*height;

			Ra = R1 + (R2 - R1)*(ip - i1);
			Rb = R3 + (R4 - R3)*(ip - i1);		
			
			Ga = G1 + (G2 - G1)*(ip - i1);
			Gb = G3 + (G4 - G3)*(ip - i1);	
			
			Ba = B1 + (B2 - B1)*(ip - i1);
			Bb = B3 + (B4 - B3)*(ip - i1);	

			// interpolate from point a to point b
			// p3 Rb p4
			// p1 Ra p2
			
			R = Ra + (Rb - Ra)*(jp - j1);
			G = Ga + (Gb - Ga)*(jp - j1);
			B = Ba + (Bb - Ba)*(jp - j1);
			
			////////////////////////////////////////////////////

			if( !( (R < 1) && (G < 1) && (B < 1) ) ) {
				*pb		= B;
				*(pb+1) = G;
				*(pb+2) = R;
			}
			
			} // end if sum > 0
			
			} // end if in range
			
			// move to next pixel in output image
			pb += 3;
			
		} // end for i
		
	} // end for j

	return 0;
}

	
int append(image &a, image &b, int ip, int jp)
{
	int i, j, ia, ja;
	int width_a, height_a, width_b, height_b;
	ibyte *pa, *pb, R, G, B;
	
	pa = a.pdata;
	width_a = a.width;
	height_a = a.height;
	
	pb = b.pdata;	
	width_b = b.width;
	height_b = b.height;
	
	// set pointer pa to beginning of the window for image b
	pa += 3*( jp*width_a + ip );
	
	for(j=0;j<height_b;j++) {		
	
		for(i=0;i<width_b;i++) {
			
			B = *pb;
			G = *(pb+1);
			R = *(pb+2);
		
			// calculate equivalent coordinates (ia,ja) in image a
			ia = ip + i;
			ja = jp + j;
			
			// only write if (ia,ja) is in range
			if( (ia >= 0) && (ia < width_a) && (ja >= 0) && (ja < height_a) ) {
					
				if( !( (R < 1) && (G < 1) && (B < 1) ) ) {
					*pa		= B;
					*(pa+1) = G;
					*(pa+2) = R;
				}
			
			} // end if in range
			
			// move to next pixel
			pa += 3;
			pb += 3;
			
		}
		
		// advance to the next line
		pa += 3*(width_a - width_b); 
		
	} // end for j
	
	return 0;
}


// 1 player, 2 player, practice, strategies, speed / difficulty level
int set_simulation_mode(int mode, int level)
{
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2
	S1->mode = mode;
	
	S1->level = level;

	return 0;
}

int set_robot_position(double x, double y, double theta)
{
	S1->P[1]->x[1] = theta;	
	S1->P[1]->x[2] = x;
	S1->P[1]->x[3] = y;

	// calculate robot output
	S1->P[1]->calculate_outputs();	

	return 0;
}


int set_opponent_position(double x, double y, double theta)
{
	if( S1->N_robot > 1 ) {
		S1->P[2]->x[1] = theta;	
		S1->P[2]->x[2] = x;
		S1->P[2]->x[3] = y;

		// calculate robot output
		S1->P[2]->calculate_outputs();		
	}
	
	return 0;
}


robot_system::robot_system(double D, double Lx, double Ly, 
	double Ax, double Ay, double alpha_max, int n_robot)
{
	int i;
	double x0, y0, theta0, vmax;
	double max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	
	// initial time
	t = 0.0;
	
	N_robot = n_robot;
	
	width  = 640;
	height = 480;

	N_obs = 0;

	// robot max wheel speed (pixels/s)
	max_speed = 100.0;

	// opponent max wheel speed (pixels/s)
	opponent_max_speed = 100.0;

	// array of pointers to robot objects
	for(i=1;i<=N_robot;i++) {
		
		if(i == 1) { // robot
			x0 = 0.3*width;
			y0 = 0.5*height;
			theta0 = 0.0;
			vmax = max_speed;
		} else if (i == 2) { // opponent
			x0 = 0.7*width;
			y0 = 0.5*height;
			theta0 = 3.14159;
			vmax = opponent_max_speed;			
		} else { // default / other robots
			x0 = 0.5*width;
			y0 = 0.5*height;
			theta0 = 3.14159/2;			
			vmax = opponent_max_speed;
		}
		
		P[i] = new robot(x0,y0,theta0,vmax);
		
		if( P[i] == NULL ) {
			cout << "\nmemory allocation error in robot_system()";
			return;
		}
		
	}

	// default actuator values
	pw_l = 1500; // us
	pw_r = 1500; // us
	pw_laser = 1500; // us
	laser = 0; // 0 or 1

	// set robot initial inputs and parameters
	for(i=1;i<=N_robot;i++) {
		P[i]->set_inputs(pw_l,pw_r,pw_laser,laser);
		P[i]->D = 121.0; // distance between wheels (pixels)
		P[i]->Lx = 31.0; // laser / gripper position (pixels)
		P[i]->Ly = 0.0; // laser / gripper position (pixels)
		P[i]->Ax = 37.0; // position of robot center relative to image center
		P[i]->Ay = 0.0; // position of robot center relative to image center		
		P[i]->alpha_max = alpha_max; // max range of laser / gripper (rad)
	}

	// take robot parameters from activate_simulation function
	P[1]->D = D; // distance between wheels (pixels)
	P[1]->Lx = Lx; // laser / gripper position (pixels)
	P[1]->Ly = Ly; // laser / gripper position (pixels)
	P[1]->Ax = Ax; // position of robot center relative to image center
	P[1]->Ay = Ay; // position of robot center relative to image center	
	P[1]->alpha_max = alpha_max; // max range of laser / gripper (rad)

	// calculate robot outputs
	for(i=1;i<=N_robot;i++) {
		P[i]->calculate_outputs();
	}	

	// default lighting condition parameters (TODO: add shadow effect, etc.)
	light = 1.0;
	light_gradient = 0.0;
	light_dir = 0.0;
	image_noise = 0.0;

	mode = 0; // default is single player mode (manual opponent)
	
	level = 1;
		
}


robot_system::~robot_system()
// class destructor
// free dynamic memory, etc.
{
	int i;
	
	// array of pointers to robot objects
	for(i=1;i<=N_robot;i++) {
		// safe delete
		if( P[i] != NULL ) {
			delete P[i];
			P[i] = NULL;
		} else {
			cout << "\nerror: NULL pointer in ~robot_system()";
		}
	}
	
}


void robot_system::sim_step(double dt)
{
	int i;	

	// * assume the inputs have already been set for each robot

	// simulate each robot in the system for one time step	
	for(i=1;i<=N_robot;i++) {
		P[i]->sim_step(dt);
	}
	
	// increment robot_system time
	t += dt;
	
}


int draw_laser(robot *P, image &rgb)
{
	double x0, y0, theta, r, dr;
	int i, j, R, G, B;

	dr = 1;

	// set laser colour
	R = 0;
	G = 255;
	B = 0;

	// get start point of the laser
	x0 = P->xg;
	y0 = P->yg;
	
	// robot theta + laser alpha
	theta = P->x[1] + P->x[4];	
	
	for(r=0;r<5000;r+=dr) {
		
		i = (int)( x0 + r*cos(theta) );
		j = (int)( y0 + r*sin(theta) );
		
		// stop loop when (i,j) goes out of range / off screen
		if( i < 3 ) break;
		if( i > rgb.width-3 ) break;
		if( j < 3 ) break;
		if( j > rgb.height-3 ) break;	
		
		draw_point_rgb_laser(rgb,i,j,R,G,B);
		
	}
	
	return 0;
}


int draw_point_rgb_laser(image &rgb, int ip, int jp, int R, int G, int B)
{
	ibyte *p;
	int i,j,w=1,pixel;

	// initialize pointer
	p  = rgb.pdata;

	if ( rgb.type != RGB_IMAGE ) {
		cout << "\nerror in draw_point_RGB: input type not valid!\n";
		return 1;
	}

	// limit out of range (i,j) values
	// NOTE: this part is important to avoid wild pointers
	if( ip < w ) ip = w;
	if( ip > rgb.width-w-1 ) ip = rgb.width-w-1;
	if( jp < w ) jp = w;
	if( jp > rgb.height-w-1 ) jp = rgb.height-w-1;

	for(i=-w;i<=w;i++) {
		for(j=-w;j<=w;j++) {
			pixel = rgb.width*(jp+j)+(ip+i);
			p[3*pixel]   = B;
			p[3*pixel+1] = G;
			p[3*pixel+2] = R;
		}
	}

	return 0;
}

