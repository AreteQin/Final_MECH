//#include "object.h"
//
//object::object(double position_x, double position_y) :
//	position_x_(position_x), position_y_(position_x) {};
//
//bool object::identify_object(image input_image,	int self_colour) {
//
//	ibyte* p = input_image.pdata;
//	int i = (int)position_x_;
//	int j = (int)position_y_;
//	// calculate the position of pointer of object center
//	p = p + (i + input_image.width * j) * 3;
//	// get the colour of object center
//	ibyte B = *p;
//	ibyte G = *(p + 1);
//	ibyte R = *(p + 2);
//	std::cout << "BGR: " << B << " " << G << " " << R << std::endl;
//	// get the colour at four directions larger than robot's wheel's size
//	int wheel_length = 12;
//	ibyte B_left = *(p - wheel_length * 3);
//	ibyte B_right = *(p + wheel_length * 3);
//	ibyte B_up = *(p + wheel_length * input_image.width * 3);
//	ibyte B_down = *(p - wheel_length * input_image.width * 3);
//	// check if is still the same color at four directions
//	// larger than robot's size
//	int max_error = 20;
//	if (abs(B - B_left) > max_error || abs(B - B_right) > max_error ||
//		abs(B - B_up) > max_error || abs(B - B_down) > max_error) {
//		object_id_ = 6;
//		return true;
//	}
//	// get the colour at four directions larger than robot's size
//	int robot_radius_in_pixel = 18;
//	B_left = *(p - robot_radius_in_pixel * 3);
//	B_right = *(p + robot_radius_in_pixel * 3);
//	B_up = *(p - robot_radius_in_pixel * input_image.width * 3);
//	B_down = *(p + robot_radius_in_pixel * input_image.width * 3);
//	// check if is still the same color at four directions
//	// larger than robot's size
//	if (self_colour == 1) {
//		if (abs(B - B_left) > max_error || abs(B - B_right) > max_error ||
//			abs(B - B_up) > max_error || abs(B - B_down) > max_error) {
//			// check the colour of center to determine is self or enemy
//			if (R > 200) {
//				if (G > 150) { object_id_ = 3; }
//				else { object_id_ = 2; }
//			}
//			else if (B > 200) { object_id_ = 4; }
//			else { object_id_ = 1; }
//		}
//	}
//	else {
//		if (abs(B - B_left) > max_error || abs(B - B_right) > max_error ||
//			abs(B - B_up) > max_error || abs(B - B_down) > max_error) {
//			// check the colour of center to determine is self or enemy
//			if (R > 200) {
//				if (G > 150) { object_id_ = 1; }
//				else { object_id_ = 4; }
//			}
//			else if (B > 200) { object_id_ = 2; }
//			else { object_id_ = 3; }
//		}
//	}
//	return true;
//}
//
//bool object::calculate_robot_direction(object self_1, object self_2) {
//	return true;
//}
//
//double object::get_position_x() {
//	return position_x_;
//}
//
//double object::get_position_y() {
//	return position_y_;
//}
//
//double object::get_theta() {
//	return theta_;
//}
//
//int object::get_id() {
//	return object_id_;
//}