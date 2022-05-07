
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
				if (G > 150) {
					object_id_ = 3;
					label_value_ = p_label[j * label_map.width + i + robot_radius];
					return 1;
				}
				else {
					object_id_ = 2;
					label_value_ = p_label[j * label_map.width + i + robot_radius];
					return 4;
				}
			}
			else if (B > 200) {
				object_id_ = 4;
				label_value_ = p_label[j * label_map.width + i + robot_radius];
				return 2;
			}
			else {
				object_id_ = 1;
				label_value_ = p_label[j * label_map.width + i + robot_radius];
				return 3;
			}
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
	/*std::cout << " rear_x: " << rear_x << std::endl;
	std::cout << " rear_y: " << rear_y << std::endl;
	std::cout << " x: " << position_x_ << std::endl;
	std::cout << " y: " << position_y_ << std::endl;
	std::cout << " position_y_ - rear_y: " << (position_y_ - rear_y) << std::endl;
	if ((position_y_ - rear_y) < 0) {
		theta = -atan2((rear_y - position_y_), (position_x_ - rear_x));
	}
	else{
		theta = atan2((position_y_ - rear_y), (position_x_ - rear_x));
	}*/
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
		//draw_point_rgb(rgb, ic[i], jc[i], 255, 0, 0);
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
			double distance = sqrt((i - objects[k].get_position_x()) * (i - objects[k].get_position_x())+
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

/// ---------------- Hiding Strategy Functions --------------------- ///
// locate the hiding point according to the enemy position and 
// the specific obstacle position
bool calculate_hiding_point(double self_x, double self_y,
	double enemy_x, double enemy_y,
	double obstacle_x, double obstacle_y,
	double& hiding_point_x, double& hiding_point_y) {
	double dx = obstacle_x - enemy_x;
	double dy = obstacle_y - enemy_y;
	double k = ((self_x - obstacle_x) * dx + (self_y - obstacle_y) * dy)
		/ ((dx * dx) + (dy * dy));
	hiding_point_x = obstacle_x + k * dx;
	hiding_point_y = obstacle_y + k * dy;
	if (abs(hiding_point_x - enemy_x) < 150) {
		hiding_point_x = obstacle_x - (enemy_x - obstacle_x);
		hiding_point_y = obstacle_y - (enemy_y - obstacle_y);
		//if (self_x > 320) {
		//	hiding_point_x = obstacle_x - (enemy_x - obstacle_x);
		//	hiding_point_y = obstacle_y - (enemy_y - obstacle_y);
		//}
		//else {
		//	hiding_point_x = obstacle_x - (enemy_x - obstacle_x);
		//	hiding_point_y = obstacle_y - (enemy_y - obstacle_y);
		//}
	}
	return true;
}

// calculate the distance between two points
double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
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
	float Radian_Temple;
	return Radian_Temple = atan2((Goal[1] - Start[1]), (Goal[0] - Start[0]));

	//if (((Goal[1] - Start[1]) > 0) && ((Goal[0] - Start[0]) > 0))
	//{
	//	return Radian_Temple;
	//}
	//if (((Goal[1] - Start[1]) > 0) && ((Goal[0] - Start[0]) < 0))
	//{
	//	return Radian_Temple + 3.1415926 / 2;
	//}
	//if (((Goal[1] - Start[1]) < 0) && ((Goal[0] - Start[0]) < 0))
	//{
	//	return Radian_Temple + 3.1415926;
	//}
	//if (((Goal[1] - Start[1]) < 0) && ((Goal[0] - Start[0]) > 0))
	//{
	//	return 2 * 3.1415926 - Radian_Temple;
	//};
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

// Functions for controller
const double pi = 3.14159265358979323846;

void inversekinematics(double v_cmd, double w_cmd, double& vr, double& vl, double D, int& pw_r, int& pw_l) {
	int max_speed = 100;

	vr = v_cmd + D * w_cmd / 2;
	vl = v_cmd - D * w_cmd / 2;

	pw_r = 1500 + vr / max_speed * 500;
	pw_l = 1500 - vl / max_speed * 500;

}

double pid(double kp, double kd, double ki, double e, double vr, double vl) {

	static double ei = 0.0;
	double dt = 0.01;

	if (abs(vr) > 100 || abs(vl) > 100) {

		ei = ei;
	}
	else
	{
		ei = ei + e * dt;
	}

	return kp * e + ki * ei;
}

void poscontroller(double xref, double yref, double x, double y, double theta, double& v_cmd, double& w_cmd) {

	double Kv, Kh, e_pos, e_angle;

	e_pos = sqrt(pow(x - xref, 2) + pow(y - yref, 2));
	e_angle = (atan2(yref - y, xref - x) - theta);

	std::cout << "\nep is " << e_pos << "\neangle is: " << e_angle;

	Kv = 0.5;
	Kh = 4;

	if (abs(e_angle) > 3.14159 / 2) {

		v_cmd = 0;
		w_cmd = Kh * e_angle;

	}
	else {
		v_cmd = Kv * e_pos;
		w_cmd = Kh * e_angle;

	}

	std::cout << "\nv is " << v_cmd << "\nw is " << w_cmd;

}

int avoid_edges(double x, double y, double x_rear, double y_rear) {

	double d[9];
	int h = 480, w = 640, k = 0;

	d[1] = h - y;
	d[2] = w - x;
	d[3] = x;
	d[4] = y;
	d[5] = h - y_rear;
	d[6] = w - x_rear;
	d[7] = x_rear;
	d[8] = y_rear;

	for (int i = 1; i < 9; i++) {
		if (d[i] < 100) {
			k = i;
		}
	}
	return k;
}

void back_up(double& vr, double& vl) {

	vr = -vr;
	vl = -vl;
}

void purepursuit(double xref, double yref, double x, double y, double theta, double x_rear,
	double y_rear, double& v_cmd, double& w_cmd, double vr, double vl, double& e_p, double& e_ang) {

	double kh = 4, kd = 0, ki = 0.005, kp = 2;
	//double e_p, e_ang, 
	double d = 1;
	double dis_c_rear;

	dis_c_rear = distance(x, y, x_rear, y_rear);

	e_p = sqrt(pow(x - xref, 2) + pow(y - yref, 2)) - 1;
	e_ang = (atan2(yref - y, xref - x) - theta);

	std::cout << "Position error is: " << e_p << "\n";
	std::cout << "Angle error is: " << e_ang << "\n";
	std::cout << "d is: " << avoid_edges(x, y, x_rear, y_rear) << "\n";

	//change hierarchy of if conditions

	if (abs(e_p) > 0.1) {

		if (abs(e_ang) > 3.14159 / 2) {

			v_cmd = 0;
			w_cmd = kh * e_ang;

		}
		// Check if robot is near edges

		else if (avoid_edges(x, y, x_rear, y_rear) > 0)
		{

			if (abs(e_ang) > 0.4) {
				v_cmd = 0;
				w_cmd = kh * e_ang;

			}
			else {
				v_cmd = pid(kp, kd, ki, e_p, vr, vl);
				w_cmd = kh * e_ang;

			}

		}
		else {
			if (e_ang < 0.1) {

				v_cmd = pid(kp, kd, ki, e_p, vr, vl);
				w_cmd = 0;

			}
			else
			{
				v_cmd = pid(kp, kd, ki, e_p, vr, vl);
				w_cmd = kh * e_ang;
			}


		}
	}
	else
	{
		v_cmd = 0;
		w_cmd = 0;
	}

}

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;

	// TODO: it might be better to put this model initialization
	// section in a separate function

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 2;

	x_obs[1] = 300; // pixels
	y_obs[1] = 200; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 400; // pixels
	y_obs[2] = 300; // pixels
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

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle_blue.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 2;
	level = 1;
	set_simulation_mode(mode, level);

	// set robot initial position (pixels) and angle (rad)
	x0 = 450;
	y0 = 150;
	theta0 = -1.71;

	set_robot_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
	// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

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

	join_player();

	// measure initial clock time
	tc0 = high_resolution_time();
	std::cout << "Start" << std::endl;
	int* OneView = new int[2];        // initialize a dynamic Array

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		// change the inputs to move the robot around
		// or change some additional parameters (lighting, etc.)
		// only the following inputs work so far
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels

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
			enemy_position_x, enemy_position_y, enemy_position_theta;

		self_position_x = self.get_position_x();
		self_position_y = self.get_position_y();
		self_position_theta = self.get_theta();
		enemy_position_x = enemy.get_position_x();
		enemy_position_y = enemy.get_position_y();
		enemy_position_theta = enemy.get_theta();
		// print the output
		std::cout << "self_position: " << self_position_x << ", "
			<< self_position_y << ", "
			<< self_position_theta 
			<< ", " << std::endl;
		std::cout << "enemy_position: " << enemy_position_x << ", "
			<< enemy_position_y << ", "
			<< enemy_position_theta 
			<< ", " << std::endl;

		//draw_point_rgb(rgb, self_position_x, self_position_y, 0, 0, 255);
		//draw_point_rgb(rgb, enemy_position_x, enemy_position_y, 0, 255, 0);

		//std::cout << "is 100, 100 free? "
			//<< check_space(objects, 320, 400) << std::endl;
		//std::cout << "is 300, 200 free? "
			//<< check_space(objects, 300, 200) << std::endl;

		// Image processing done ---------------------------------------------

		// Hiding Strategy ---------------------------------------------------
		// input: self state, enemy state, all obstacles
		// output: expected state
		double expected_x, expected_y, expected_theta;
		calculate_expected_position(self_position_x, self_position_y, self_position_theta,
			enemy_position_x, enemy_position_y, enemy_position_theta, objects,
			expected_x, expected_y, expected_theta);
		//draw_point_rgb(rgb, expected_x, expected_y, 255, 0, 0);
		// Hiding Strategy done ---------------------------------------------------

		 //Path planning ---------------------------------------------------------
		 //Part 2.---------------------Track object for Path planning----------------------Start
		 //Author  : Xiaobo Wu 
		 //Data    : 2022.04.10 
		int A_Global_Start[2] = { self_position_x, self_position_y };                      //initiallze a dynamic input parameter
		int B_Global_End[2] = { expected_x, expected_y };
		PathTrack(OneView, A_Global_Start, B_Global_End, 3.1415926 / 19, 40, objects);
		//PathTrack(OneView, A_Global_Start, B_Global_End, 3.1415926 / 19, 20, objects);     // realize the one-step viewpoint path track
		// Path planning done -------------------------------------------------------

		// Controller -----------------------------------------------------------
		double x_rear, y_rear, x_ref, y_ref;
		double e_p, e_ang, v_cmd, w_cmd, vr, vl;

		/*x_ref = 150;
		y_ref = 400;*/

		x_rear = self_rear.get_position_x();
		y_rear = self_rear.get_position_y();

		std::cout << "OneView: " << OneView[0] << ", " << OneView[1] << std::endl;
		//if (check_space(objects, OneView[0], OneView[1])) {
		//if (check_space(objects, expected_x, expected_y)) {
		//	purepursuit(expected_x, expected_y, self_position_x, self_position_y, self_position_theta, x_rear, y_rear, v_cmd, w_cmd, vr, vl, e_p, e_ang);
		//	inversekinematics(v_cmd, w_cmd, vr, vl, D, pw_r, pw_l);
		//}
		//else {
		//	purepursuit(width/2, height/2, self_position_x, self_position_y, self_position_theta, x_rear, y_rear, v_cmd, w_cmd, vr, vl, e_p, e_ang);
		//	inversekinematics(v_cmd, w_cmd, vr, vl, D, pw_r, pw_l);
		//}
		purepursuit(OneView[0], OneView[1], self_position_x, self_position_y, self_position_theta, x_rear, y_rear, v_cmd, w_cmd, vr, vl, e_p, e_ang);
		inversekinematics(v_cmd, w_cmd, vr, vl, D, pw_r, pw_l);

		// Controller done ---------------------------------------------------------

		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);

		// NOTE: only one program can call view_image()
		//view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();

	deactivate_simulation();

	std::cout << "\ndone.\n";

	return 0;
}
