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

void purepursuit(double xref, double yref, double x, double y, double theta, double x_rear, 
	double y_rear, double& v_cmd, double& w_cmd, double vr, double vl) {

	double kh = 4, kd = 0, ki = 0.005,kp = 4;
	double e_p, e_ang;
	int d = 1;

	e_p = sqrt(pow(x - xref, 2) + pow(y- yref, 2)) - d;
	e_ang = (atan2(yref - y, xref - x) - theta);

	std::cout << "Position error is: " << e_p << "\n";
	std::cout << "Angle error is: " << e_ang << "\n";

	if (abs(e_p) > 1) {
		
		if (abs(e_ang) > 3.14159 / 2) {

			v_cmd = 0;
			w_cmd = kh * e_ang;

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

void edge_avoidance(double x, double y, double theta, double& v_cmd, double& w_cmd, double vr, double vl) {

	double e_ang, e_p, d = 1, kh = 4, kp = 4, kd = 0, ki = 0.005;

	std::cout << "Using edge_avoidance!!!\n";
	e_p = sqrt(pow(x - 320, 2) + pow(y - 240, 2)) - d;
	e_ang = (atan2(320 - y,240 - x) - theta);

	if (abs(e_ang) > 3.14159 / 2) {

		v_cmd = 0;
		w_cmd = kh * e_ang;

	}
	else
	{
		v_cmd = 100;
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
		if (d[i] < 120) {
			k = i;
		}
	}
	
	std::cout << "d is :" << k << "\n";
	return k;
} 

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}