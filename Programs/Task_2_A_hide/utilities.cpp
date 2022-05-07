#include "utilities.h"
#include <iostream>
#include <cmath>

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

void purepursuit(double xref, double yref, double x, double y, double theta, double x_rear, 
	double y_rear, double& v_cmd, double& w_cmd, double vr, double vl, double &e_p, double &e_ang) {

	double kh = 4, kd = 0, ki = 0.005,kp = 2;
	//double e_p, e_ang, 
	double d = 1;
	double dis_c_rear;

	dis_c_rear = distance1(x, y, x_rear, y_rear);

	e_p = sqrt(pow(x - xref, 2) + pow(y- yref, 2)) - 1;
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
		
		else if (avoid_edges(x, y, x_rear, y_rear) > 0 )
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
		if (d[i] < 80) {
			k = i;
		}
	}
	return k;
} 

void back_up(double& vr, double& vl) {

	vr = -vr;
	vl = -vl;
}

double distance1(double x1, double y1, double x2, double y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}