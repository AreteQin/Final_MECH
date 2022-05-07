#pragma once


void inversekinematics(double v_cmd, double w_cmd, double& vr, double& vl, double D, int& pw_r, int& pw_l);

double pid(double kp, double kd, double ki, double e, double vr, double vl);

void purepursuit(double xref, double yref, double x, double y, double theta, double x_rear, 
	double y_rear, double& v_cmd, double& w_cmd, double vr, double vl);

void edge_avoidance(double x, double y, double theta, double& v_cmd, double& w_cmd, double vr, double vl);

int avoid_edges(double x, double y, double x_rear, double y_rear);

double distance(double x1, double y1, double x2, double y2);