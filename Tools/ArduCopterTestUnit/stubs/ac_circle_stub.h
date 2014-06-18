/*
 * ac_circle_stub.h
 *
 *  Created on: 17 juin 2014
 *      Author: valentin
 */

#ifndef AC_CIRCLE_STUB_H_
#define AC_CIRCLE_STUB_H_

class AC_Circle
{

public:
	AC_Circle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control) {}

	void set_center(const Vector3f& center) {};

	float get_angle_total() const { return 0.1; }

};

#endif /* AC_CIRCLE_STUB_H_ */
