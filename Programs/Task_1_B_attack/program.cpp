
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

#include "utilities.h"


extern robot_system S1;

// define a class to represent objects
// object_id_(1: self robot head, 2 : self robot rear
//            3: enemy head, 4: enemy rear
//            5: obstacles, 6: wheels)
class object {
public:
	object();
	object(double position_x, double position_y);
	// identify the object:
	// *input: rgb image, self robot colour, center of object
	// *return: object_id_
	int identify_object(image input_RGB_image,
		image label_map,
		int self_colour);
	bool calculate_robot_theta(object rear);
	double get_position_x();
	double get_position_y();
	double get_theta();
	int get_id();
	int get_label_value();
private:
	int object_id_, label_value_;
	double position_x_, position_y_, theta_;
};

object::object() {
	position_x_ = 0;
	position_y_ = 0;
	theta_ = 0;
}

object::object(double position_x, double position_y) :
	position_x_(position_x), position_y_(position_y) {};

int object::identify_object(image input_RGB_image,
							image label_map,
							int self_colour) {
	i2byte* p_label = (i2byte*)label_map.pdata;
	ibyte* p = input_RGB_image.pdata;
	int i = (int)position_x_;
	int j = (int)position_y_;
	//std::cout << "label: " << p_label[j * label_map.width + i] << std::endl;
	// calculate the position of pointer of object center
	int wheel_length = 4;
	ibyte* p_center = p + (i + input_RGB_image.width * j) * 3;
	ibyte* p_left = p + (i - wheel_length + input_RGB_image.width * j) * 3;
	ibyte* p_right = p + (i + wheel_length + input_RGB_image.width * j) * 3;
	ibyte* p_up = p + (i + input_RGB_image.width * (j + wheel_length)) * 3;
	ibyte* p_down = p + (i + input_RGB_image.width * (j - wheel_length)) * 3;
	// get the colour of object center
	if (i> input_RGB_image.width-16 || i < 16 ||
		j> input_RGB_image.height - 16 || j<16) {
		std::cout << "out of sight!----------------------" << std::endl;
		return 0;
	}
	ibyte B = *p_center;
	ibyte G = *(p_center + 1);
	ibyte R = *(p_center + 2);
	// get the colour at four directions larger than robot's wheel's size
	ibyte B_left = *p_left;
	ibyte B_right = *p_right;
	ibyte B_down = *p_down;
	ibyte B_up = *p_up;
	// check if is still the same colour at four directions
	// larger than robot's size
	int max_error = 15;
	int left_error = abs(B_left - B);
	int right_error = abs(B_right - B);
	int up_error = abs(B_up - B);
	int down_error = abs(B_down - B);
	//std::cout << "errors: " << left_error <<" "
	//						<< right_error << " "
	//						<< up_error << " "
	//						<< down_error << std::endl;
	if (left_error > max_error || right_error > max_error ||
		up_error > max_error || down_error > max_error) {
		//std::cout << "found a wheel" << std::endl;
		object_id_ = 6;
		label_value_ = p_label[j * label_map.width + i];
		return 6;
	}
	// get the colour at four directions larger than robot's size
	int robot_radius = 15;
	p_left = p + (i - robot_radius + input_RGB_image.width * j) * 3;
	p_right = p + (i + robot_radius + input_RGB_image.width * j) * 3;
	p_up = p + (i + input_RGB_image.width * (j + robot_radius)) * 3;
	p_down = p + (i + input_RGB_image.width * (j - robot_radius)) * 3;
	B_left = *p_left;
	B_right = *p_right;
	B_down = *p_down;
	B_up = *p_up;
	left_error = abs(B_left - B);
	right_error = abs(B_right - B);
	up_error = abs(B_up - B);
	down_error = abs(B_down - B);
	//std::cout << "errors: " << left_error << " "
	//	<< right_error << " "
	//	<< up_error << " "
	//	<< down_error << std::endl;
	// check if is still the same color at four directions
	// larger than robot's size
	if (self_colour == 1) {
		if (left_error > max_error || right_error > max_error ||
			up_error > max_error || down_error > max_error) {
			// check the colour of center to determine is self or enemy
			if (R > 200) {
				if (G > 150) { 
					object_id_ = 3; 
					label_value_ = p_label[j * label_map.width + i + robot_radius];
					return 3; 
				}
				else { 
					object_id_ = 2;
					label_value_ = p_label[j * label_map.width + i + robot_radius]; 
					return 2; }
			}
			else if (B > 200) { object_id_ = 4; 
			label_value_ = p_label[j * label_map.width + i + robot_radius]; 
			return 4; }
			else { object_id_ = 1; 
			label_value_ = p_label[j * label_map.width + i + robot_radius]; 
			return 1; }
		}
	}
	else {
		if (left_error > max_error || right_error > max_error ||
			up_error > max_error || down_error > max_error) {
			// check the colour of center to determine is self or enemy
			if (R > 200) {
				if (G > 150) { object_id_ = 1; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 1; }
				else { object_id_ = 4; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 4; }
			}
			else if (B > 200) { object_id_ = 2; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 2; }
			else { object_id_ = 3; label_value_ = p_label[j * label_map.width + i + robot_radius]; return 3; }
		}
	}
	//std::cout << "found an obstacle" << std::endl;
	object_id_ = 5;
	//label_value_ = p_label[j * label_map.width + i];
	return 5;
}

bool object::calculate_robot_theta(object rear) {
	double rear_x = rear.get_position_x();
	double rear_y = rear.get_position_y();
	double theta;
	

	theta = atan2((position_y_ - rear_y), (position_x_ - rear_x));
	theta_ = theta;
	return true;
	
	
}

double object::get_position_x() {
	return position_x_;
}

double object::get_position_y() {
	return position_y_;
}

double object::get_theta() {
	return theta_;
}

int object::get_id() {
	return object_id_;
}

int object::get_label_value() {
	return label_value_;
}

// get positions of two robots and map which includes all obstacles,
// return the number of found objects
int get_positions_from_image(image rgb, int self_colour,
	std::vector<object> &objects, image &label_map) {
	image temp_image, map;
	temp_image.type = GREY_IMAGE;
	temp_image.width = rgb.width;
	temp_image.height = rgb.height;
	map.type = GREY_IMAGE;
	map.width = rgb.width;
	map.height = rgb.height;
	allocate_image(map);
	allocate_image(temp_image);
	copy(rgb, temp_image);
	map.type = GREY_IMAGE;
	scale(temp_image, map);
	lowpass_filter(map, temp_image);
	threshold(temp_image, map, 180);
	invert(map, temp_image);
	erode(temp_image, map); // denoise
	dialate(map, temp_image);
	//copy(temp_image, rgb);
	//view_rgb_image(rgb);
	//pause();
	// label objects
	int nlabel;
	label_image(temp_image, label_map, nlabel);
	std::cout << "found " << nlabel << " objects" << std::endl;
	double ic[9], jc[9];
	for (int i = 1; i <= nlabel; i++) {
		centroid(temp_image, label_map, i, ic[i], jc[i]);
		object obj(ic[i], jc[i]);
		//std::cout << "object " << i << " position: " << ic[i] << " " << jc[i] << std::endl;
		objects.push_back(obj);
		objects[i-1].identify_object(rgb, label_map, self_colour);
		std::cout << "id: x,y,label: " << objects[i - 1].get_id() << ": " <<
			ic[i] << " " << jc[i] << " " << objects[i - 1].get_label_value()<< std::endl;
		//draw_point_rgb(rgb, ic[i], jc[i], 0, 0, 255);
	}

	free_image(temp_image);
	free_image(map);
	return true;
}

// check whether a pixel is part of obstacls
// return true if it is free
bool check_space(std::vector<object> objects, int i, int j) {
	double obstacle_radius = 80;
	if (i < 20 || i>620 || j < 20 || j>460) {
		return false;
	}
	for (int k = 0; k < objects.size(); k++) {
		if (objects[k].get_id() == 5) {
			double distance = sqrt((i - objects[k].get_position_x()) * (i - objects[k].get_position_x()) +
				(j - objects[k].get_position_y()) * (j - objects[k].get_position_y()));
			if (distance < obstacle_radius) {
				return false;
			}
		}
	}
	return true;
}

bool check_space_laser(std::vector<object> objects, int i, int j) {
	double obstacle_radius = 50;
	if (i < 20 || i>620 || j < 20 || j>460) {
		return false;
	}
	for (int k = 0; k < objects.size(); k++) {
		if (objects[k].get_id() == 5) {
			double distance = sqrt((i - objects[k].get_position_x()) * (i - objects[k].get_position_x()) +
				(j - objects[k].get_position_y()) * (j - objects[k].get_position_y()));
			if (distance < obstacle_radius) {
				return false;
			}
		}
	}
	return true;
}

// find out objects represent self and enemy robots from all objects
bool get_robots(std::vector<object> objects, object& self, object& self_rear,
				object& enemy, object& enemy_rear) {
	for (int i = 0; i < objects.size(); i++) {
		if (objects[i].get_id() == 1) {
			self = objects[i];
		}
		if (objects[i].get_id() == 3) {
			enemy = objects[i];
		}
		if (objects[i].get_id() == 2) {
			self_rear = objects[i];
		}
		if (objects[i].get_id() == 4) {
			enemy_rear = objects[i];
		}
	}
	return true;
}

// locate the hiding point according to the enemy position and 
// the specific obstacle position
bool calculate_hiding_point(double self_x, double self_y,
	double enemy_x, double enemy_y,
	double obstacle_x, double obstacle_y,
	double &hiding_point_x, double &hiding_point_y) {
	double dx = obstacle_x - enemy_x;
	double dy = obstacle_y - enemy_y;
	double k = ((self_x - obstacle_x) * dx + (self_y - obstacle_y) * dy)
		/ ((dx * dx) + (dy * dy));
	hiding_point_x = obstacle_x + k * dx;
	hiding_point_y = obstacle_y + k * dy;
	return true;
}


bool calculate_expected_position(double self_x, double self_y, double self_theta,
	double enemy_x, double enemy_y, double enemy_theta, std::vector<object> objects,
	double &expected_x, double &expected_y, double &expected_theta) {
	std::vector<double> points_x, points_y;
	for (int i = 0; i < objects.size(); i++) {
		if (objects[i].get_id() == 5) {
			double point_x, point_y;
			calculate_hiding_point(self_x, self_y, enemy_x, enemy_y,
				objects[i].get_position_x(), objects[i].get_position_y(),
				point_x, point_y);
			points_x.push_back(point_x);
			points_y.push_back(point_y);
		}
	}
	double min_distance=2000.0;
	for (int i = 0; i < points_x.size(); i++) {
		double distance_ = distance(points_x[i], points_y[i], self_x, self_y);
		std::cout << "distance: " << distance_ << std::endl;
		if (distance_ < min_distance) {
			min_distance = distance_;
			expected_x = points_x[i];
			expected_y = points_y[i];
			std::cout << "hiding_point: " << points_x[i] << ", " << points_y[i] << std::endl;
			std::cout << "distance: " << distance_ << std::endl;
		}
	}
	if (enemy_theta > 0) {
		expected_theta = enemy_theta - 3.14;
		return true;
	}
	expected_theta = enemy_theta + 3.14;
	return true;
}

////part 1------------The Funciton Defination for Path planning------------Starts
// Author  : Xiaobo Wu
// Data    : 2022.04.11/edition-1
// Function: path planning part include four subfunctions: 
// 
// 1.SamplingArray: Create Sample point in Body coordinate system;
// 2.RotationArray: Transform Sample point from Body coordinate system to Globle coordinate system
// 3.Get_Angle_Rotation: Get part angle for transform of rotation Array
// 4.PathTrack : When you know the start position(the current position of car) and end/Goal position(the position where you want car to go),You can use this function


//------------------1.SamplingArray: Create Sample point from Body coordinate system-------Start
void SamplingArray(float Sampling_points[(270 / 5 + 1) * 3], float Mini_Radian, float S_R)
/////////////////////////////////////////////////////////////////////////////////////
// Input :  Mini_Radian: Minimum radian interval for search
//          S_R        : Sample Search Step size  
// Output:  Sampling point and turn angle in body coordinate system 
// 
// Notes for Output: from first element to third element is the first sampling group ;
// and from fourth element to sixthe element is the second sampling group in array Sampling_points;
/////////////////////////////////////////////////////////////////////////////////////
{
	Sampling_points[0] = S_R;
	Sampling_points[1] = 0;
	Sampling_points[2] = 0;
	//cout << Sampling_points[0] << Sampling_points[1] << Sampling_points[2];
	//cout << "\n";
	int i_R = 1;
	for (int ii = 1; ii < (270 / 5); ii++)
	{
		Sampling_points[3 * ii] = S_R * cos(Mini_Radian * i_R);
		Sampling_points[3 * ii + 1] = S_R * sin(Mini_Radian * i_R);
		Sampling_points[3 * ii + 2] = Mini_Radian * i_R;

		Sampling_points[3 * ii + 3] = +Sampling_points[3 * ii];
		Sampling_points[3 * ii + 4] = -Sampling_points[3 * ii + 1];
		Sampling_points[3 * ii + 5] = -Sampling_points[3 * ii + 2];

		//		cout << Sampling_points[3 * ii] <<" ;" << Sampling_points[3 * ii + 1] << " ;" << Sampling_points[3 * ii + 2];
		//		cout << "\n";

		ii++;
		i_R = i_R + 1;
	}
}
//------------------2.RotationArray: Transform Sample point------------------
int TF_X(float Reason[2], int Translation_x, float Radian)
{
	// Input:
	//	Reason[2]:      Position to be converted  in the body coordinate system
	//	Translation[2]: Translation or the current position of the object in map coordinate system
	//	Radian:         Rotation angle from body coordinate system to map coordinate system(clockwise)
	// Output:
	//	Result_x:       Is the x value be converted in the map coordinate system
	int Result_x;
	Result_x = cos(Radian) * Reason[0] - sin(Radian) * Reason[1] + Translation_x;
	return Result_x;
}
int TF_Y(float Reason[2], int Translation_y, float Radian)
{
	// Input:
	//	Reason[2]:      Position to be converted  in the body coordinate system
	//	Translation[2]: Translation or the current position of the object in map coordinate system
	//	Radian:         Rotation angle from body coordinate system to map coordinate system(clockwise)
	// Output:
	//	Result_y:       Is the y value be converted in the map coordinate system
	int Result_y;
	Result_y = sin(Radian) * Reason[0] + cos(Radian) * Reason[1] + Translation_y;
	return Result_y;
}
//----------------- 3.Get the angle from Start_A(Current)and Goal_B in the map coordinate system----------
float Get_Angle_Rotation(int Start[2], int Goal[2])
/////////////////////////////////////////////////////////////////////////////////////
// Input: Start(Current)Point  A and Goal Point B
// Output: the angle of from A to B (0 <= angle < 2 * 3.1415926)
/////////////////////////////////////////////////////////////////////////////////////
{
	float Radian_Temple,diff;
	diff = (Goal[0] - Start[0]);

	if (diff < 0.00001) {
		diff = 0.00001;
	}
	else
	{
		Radian_Temple = atan(abs((Goal[1] - Start[1]) / diff));
	}

	if (((Goal[1] - Start[1]) > 0) && ((Goal[0] - Start[0]) > 0))
	{
		return Radian_Temple;
	}
	if (((Goal[1] - Start[1]) > 0) && ((Goal[0] - Start[0]) < 0))
	{
		return Radian_Temple + 3.1415926 / 2;
	}
	if (((Goal[1] - Start[1]) < 0) && ((Goal[0] - Start[0]) < 0))
	{
		return Radian_Temple + 3.1415926;
	}
	if (((Goal[1] - Start[1]) < 0) && ((Goal[0] - Start[0]) > 0))
	{
		return 2 * 3.1415926 - Radian_Temple;
	};
}
////1.part-------The Funciton Defination for Path planning --------End
void PathTrack(int OneViewPoint[2], int A_Global_Start[2], int B_Global_End[2], float Mini_Radian, float S_R, std::vector<object>& objects)

//Input 1 : A_Global_Start[2]:the start/current position of car; 
//Input 2 : B_Global_End[2]:  end/Goal position(the position where you want car to go),
//Input 3 : Mini_Radian: Minimum radian interval for search
//Input 4 : S_R        : Sample Search Step size  
//Input 5 : std::vector<object>& objects  : A class form teamclass qiaomneng Qin
//Output  : OneViewPoint[2]:  a step viewpoint of path planning from Start point to End point(maybe you think it is a input actually it is ready for dynamic memory block please see gordan`s course in week 4)
//
//
{
	float* S = new float[(270 / 5 + 1) * 3];       // initialize a dynamic Array
	SamplingArray(S, Mini_Radian, S_R);         // Sampling points be stored in S array in the body coodination
	// Intermediate variable
	float Sample_Point_Body[2];
	float Sample_Theta_Body;
	int Sample_Point_Global[2];
	float Sample_Theta_Global;
	float Radian_Get;
	for (int j = 0; j < 270 / 5 + 1; j++)
	{
		//initialize the point to be transform
		Sample_Point_Body[0] = S[j];
		Sample_Point_Body[1] = S[j + 1];
		Sample_Theta_Body = S[j + 2];
		// Initialize the Rotation angle value from A TO B point in the global coordinate system
		Radian_Get = Get_Angle_Rotation(A_Global_Start, B_Global_End) + Sample_Theta_Body;
		// Transform body coordinate system to global coordinte system
		Sample_Point_Global[0] = TF_X(Sample_Point_Body, A_Global_Start[0], Radian_Get);
		Sample_Point_Global[1] = TF_Y(Sample_Point_Body, A_Global_Start[1], Radian_Get);
		if (check_space(objects, Sample_Point_Global[0], Sample_Point_Global[1]) == true)
		{
			/* be used to rebug , do not move it
					cout << "get veiw point/ Global positon:";
			cout << Sample_Point_Global[0] << ";" << Sample_Point_Global[1];
			cout << "\n";
			cout << " And Body Position And Theta :";
			cout << Sample_Point_Body[0] << ";" << Sample_Point_Body[1] << ";" << 180 * Sample_Theta_Body / 3.1415926;
			cout << "\n";
			cout << "Position A , Angle of A TO B Theta, Angle of Rotation:";
			cout << A_Global_Start[0] << ";"<< A_Global_Start[1];
			cout << ";" << 180 * Get_Angle_Rotation(A_Global_Start, B_Global_End) / 3.1415926 << ";" << 180 * Radian_Get / 3.1415926;
			cout << "\n";
			*/

			OneViewPoint[0] = Sample_Point_Global[0];                           // update x label of start point 
			OneViewPoint[1] = Sample_Point_Global[1];                           // update y label of start point
			//Check_Local_Min[kkk] = { A_Global_Start[0], A_Global_Start[1] };
			//kkk++;

			break;
		}
		j = j + 2;
	}

}

////////////////////// Attack functions ////////////////////////////////////////////////////////////////

const double pi = 3.14159265358979323846;

void laser_track(double x, double y, double theta, double enemy_x, double enemy_y, double erear_x, double erear_y, int& pw_laser, double& e_ang, double& alpha) {

	double dc, dr, gamma;

	dc = distance(x, y, enemy_x, enemy_y);
	dr = distance(x, y, erear_x, erear_y);

	alpha = ((double)pw_laser - 1000) / 1000 * pi;

	if (dc <= dr) {
		
		gamma = atan2(enemy_y - y, enemy_x - x);

		if (pw_laser <= 1500) {

			e_ang = pi / 2 - alpha + (gamma - theta);
			pw_laser += (int)(e_ang / pi * 1000);

		}
		else if (pw_laser > 1500) {

			e_ang = alpha - pi / 2 - (gamma - theta);
			pw_laser += (int)(e_ang / pi * 1000);

		}

	}
	else if (dc > dr)
	{
		gamma = atan2(erear_y - y, erear_x - x);

		if (pw_laser <= 1500) {

			e_ang = pi / 2 - alpha + (gamma - theta);
			pw_laser += (int)(e_ang / pi * 1000);

		}
		else if (pw_laser > 1500) {

			e_ang = alpha - pi / 2 - (gamma - theta);
			pw_laser += (int)(e_ang / pi * 1000);

		}

	}
	
}

int free_path(image rgb, std::vector<object> objects, double laser_x, double laser_y, double theta, double enemy_x, double enemy_y, double erear_x, double erear_y) {

	int x_path, y_path, path_free = 1;
	double r, phi, d_front, d_rear;

	d_front = distance(laser_x, laser_y, enemy_x, enemy_y);
	d_rear = distance(laser_x, laser_y, erear_x, erear_y);

	if (d_front < d_rear) {

		phi = atan2(enemy_y - laser_y, enemy_x - laser_x);
		r = d_front;
	}
	else
	{
		phi = atan2(erear_y - laser_y, erear_x - laser_x);
		r = d_rear;
	}


	for (int d = 0; d < r; d++) {

		x_path = (int)(laser_x + d * cos(phi));
		y_path = (int)(laser_y + d * sin(phi));

		//draw_point_rgb(rgb, x_path, y_path, 255, 0, 0);

		if (check_space_laser(objects, x_path, y_path) == 0) {
			//path crosses an obstacle
			path_free = 0;

		}
	
	}

	return path_free;
}

////////////////////// End of Attack functions ////////////////////////////////////////////////////////////////

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1,v_cmd,w_cmd,vr,vl;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;


	std::ofstream fout("sim1.csv"); // output file

	fout << std::scientific;


	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 2;

	x_obs[1] = 300; // pixels
	y_obs[1] = 200; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 320; // pixels
	y_obs[2] = 280; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
		
	std::cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();



	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_B.bmp", "robot_A.bmp", "background.bmp", "obstacle_black.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);
	
	mode = 1;
	level = 1;
	set_simulation_mode(mode, level);

	///////////////////// set robot initial position (pixels) and angle (rad)//////////////////////
	 
	x0 = 150;
	y0 = 180;
	theta0 = 1;
	set_robot_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
	// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	vr = 0;
	vl = 0;

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	image rgb;
	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	// allocate memory for the images
	allocate_image(rgb);

	wait_for_player();

	// measure initial clock time
	tc0 = high_resolution_time();
	std::cout << "Start" << std::endl;

	double laser_pre_error = 10000;
	double theta_1 = 0;

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		//   Image processing --------------------------------------------------

		// The colour of our own robot which should be known, 1 represents A
		// 2 represents B
		int self_colour = 2;
		std::vector<object> objects;
		image label_map;
		label_map.type = LABEL_IMAGE;
		label_map.width = rgb.width;
		label_map.height = rgb.height;
		allocate_image(label_map);

		int nlabel = get_positions_from_image(rgb, self_colour, objects, 
											label_map);

		object self,self_rear,enemy,enemy_rear;
		get_robots(objects,self,self_rear,enemy,enemy_rear);
		self.calculate_robot_theta(self_rear);
		enemy.calculate_robot_theta(enemy_rear);

		double self_position_x, self_position_y, self_position_theta,
			enemy_position_x, enemy_position_y, enemy_position_theta,
			laser_x, laser_y,prev_theta;
		double x_rear, y_rear;


		self_position_x = self.get_position_x();
		self_position_y = self.get_position_y();
		self_position_theta = self.get_theta();

		x_rear = self_rear.get_position_x();
		y_rear = self_rear.get_position_y();

		if (x_rear == 0 && y_rear == 0) {

			self_position_theta = theta_1;
		}
		else
		{
			theta_1 = self_position_theta;
		}

		//draw_point_rgb(rgb, self_position_x, self_position_y, 0, 0, 255);
		//draw_point_rgb(rgb, self_rear.get_position_x(), self_rear.get_position_y(), 0, 0, 255);

		laser_x = self_position_x + 31 * cos(self_position_theta);
		laser_y = self_position_y + 31 * sin(self_position_theta);
		
		//draw_point_rgb(rgb, laser_x, laser_y, 0, 255, 0);

		enemy_position_x = enemy.get_position_x();
		enemy_position_y = enemy.get_position_y();
		enemy_position_theta = enemy.get_theta();

		//draw_point_rgb(rgb, enemy_position_x, enemy_position_y, 0, 0, 255);
		//draw_point_rgb(rgb, enemy_rear.get_position_x(), enemy_rear.get_position_y(), 0, 0, 255);


		// print the output
		std::cout << "self_position: " << self_position_x << ", "
			<< self_position_y << ", "
			<< self_position_theta
			<< ", " << std::endl;
		std::cout << "enemy_position: " << enemy_position_x << ", "
			<< enemy_position_y << ", "
			<< enemy_position_theta
			<< ", " << std::endl;

		double self_centroids, enemy_centroids;

		self_centroids = distance(self_position_x, self_position_y, self_rear.get_position_x(), self_rear.get_position_y());
		enemy_centroids = distance(enemy_position_x, enemy_position_y, enemy_rear.get_position_x(), enemy_rear.get_position_y());
		

		/// Part 2.---------------------Track object for Path planning----------------------Start
		/// Author  : Xiaobo Wu 
		/// Data    : 2022.04.10 
		int A_Global_Start[2] = { self_position_x, self_position_y };                      //initiallze a dynamic input parameter
		int B_Global_End[2] = { enemy_position_x, enemy_position_y };
		int* OneView = new int[2];                                                         // initialize a dynamic Array
		PathTrack(OneView, A_Global_Start, B_Global_End, 3.1415926 / 19, 30, objects);     // realize the one-step viewpoint path track
		//draw_point_rgb(rgb, OneView[0], OneView[1], 225, 0, 0);                            // draw the view point to track/draw the goal point

		////part 2.------------------Track object for Path planning-----------------END


		// Image processing done ---------------------------------------------

		///////////////////////Controlling/////////////////////////////////////////
		
		double l_ang,laser_alpha;

		//Check for obstacles
		if (check_space(objects, B_Global_End[0], B_Global_End[1])) {
			purepursuit(OneView[0], OneView[1], self_position_x, self_position_y, self_position_theta, x_rear, y_rear, v_cmd, w_cmd, vr, vl);
			inversekinematics(v_cmd, w_cmd, vr, vl, D, pw_r, pw_l);
		}

		else if (avoid_edges(self_position_x, self_position_y, x_rear, y_rear) > 4 && self_position_x != 0 && x_rear != 0)
		{
			pw_r = 2000;
			pw_l = 1000;
		}

		else if (avoid_edges(self_position_x, self_position_y, x_rear, y_rear) < 4 && avoid_edges(self_position_x, self_position_y, x_rear, y_rear) > 0 && self_position_x != 0 && x_rear != 0)
		{
			pw_r = 1000;
			pw_l = 2000;
		}

		else {
			pw_r = 1000;
			pw_l = 2000;
		}
		
		//Laser servo tracking enemy function
		laser_track(laser_x, laser_y, self_position_theta, enemy_position_x, enemy_position_y, enemy_rear.get_position_x(), enemy_rear.get_position_y(), pw_laser, l_ang, laser_alpha);
		
		//Checking for saturation values
		if (pw_laser > 2000 || pw_laser < 1000) {

			pw_laser = 1500;
		}

		/////////// Printing data /////////////////

		std::cout << "time is: " << tc << "\n";
		std::cout << "v_cmd is: " << v_cmd << "\n";
		std::cout << "w_cmd is: " << w_cmd << "\n";
		//std::cout << "self centroids is: " << self_centroids << "\n";
		//std::cout << "self_rear x is: " << x_rear << "\n";
		//std::cout << "self_rear y is: " << y_rear << "\n";
		//std::cout << "enemy centroids is: " << enemy_centroids << "\n";
		//std::cout << "laser error is: " << l_ang  << "\n";
		//std::cout << "Alpha is: " << laser_alpha * 180 / pi << "\n";
		//std::cout << "pw_laser is: " << pw_laser << "\n";
		//std::cout << "laser is: " << laser << "\n";
		//std::cout << "laser before: " << laser_pre_error << "\n";
		
		double phi;

		phi = atan2(laser_y - self_position_y, laser_x - self_position_x);

		if (free_path(rgb, objects, laser_x, laser_y, self_position_theta, enemy_position_x, enemy_position_y, enemy_rear.get_position_x(), enemy_rear.get_position_y())) {

			std::cout << "PATH IS FREE!" << "\n";

			if (abs(l_ang) < 0.05 && abs(laser_pre_error) < 0.05 && self_centroids < 90 && enemy_centroids < 90 && abs(phi - self_position_theta) < 0.001 && x_rear != 0) {
				
				laser = 1;
				std::cout << "LASER ERROR IS SMALL" << "\n";
				std::cout << "laser error is: " << l_ang << "\n";

			}


		}
		else
		{
			std::cout << "PATH IS NOT FREE! " << "\n";
		}

		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);

		laser_pre_error = l_ang;
		
		std::cout << "\n";
		//robot state
		fout << tc << "," << self_position_theta << "," << self_position_x << "," << self_position_y << ",";
		//controller parameters
		fout << pw_r << "," << pw_l << "," << vr << "," << vl << "," << v_cmd << "," << w_cmd << "," << l_ang << ",";
		//enemy state
		fout << enemy_position_theta << "," << enemy_position_x << "," << enemy_position_y;

		fout << "\n";

		// NOTE: only one program can call view_image()
		view_rgb_image(rgb);
		delete[] OneView;
		// don't need to simulate too fast
		//Sleep(1); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();

	deactivate_simulation();


	return 0;
}
