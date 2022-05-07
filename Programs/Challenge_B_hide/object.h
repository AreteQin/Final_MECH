//#pragma once
//#include <cmath>
//#include <iostream>
//
//#include "image_transfer.h"
//#include "vision_simulation.h"
//#include "vision.h"
//
//// define a class to represent objects
//// object_id_(1: self robot center, 2 : self robot direction
////            3: enemy center, 4: enemy direction 
////            5: obstacles, 6: wheels)
//class object {
//public:
//	object(double position_x, double position_y);
//	// identify the object:
//	// return true if success,
//	// input: rgb image, self robot colour, center of object
//	// output: object_id_
//	bool identify_object(image input_image, int self_colour);
//	bool calculate_robot_direction(object self_1, object self_2);
//	double get_position_x();
//	double get_position_y();
//	double get_theta();
//	int get_id();
//private:
//	int object_id_;
//	double position_x_, position_y_, theta_;
//};
//
