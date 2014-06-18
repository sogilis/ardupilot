/*
 * ac_wpnav_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef AC_WPNAV_STUB_H_
#define AC_WPNAV_STUB_H_

class AC_WPNav {
public:

	bool waypopint_destination_is_reached;

	// spline segment end types enum
	enum spline_segment_end_type {
	    SEGMENT_END_STOP = 0,
	    SEGMENT_END_STRAIGHT,
	    SEGMENT_END_SPLINE
	};

	float speed_in_cms;
	void set_desired_alt(float desired_alt) {}
	void set_speed_xy(float speed_cms) {speed_in_cms = speed_cms;}
	void set_fast_waypoint(bool fast) {}
	void get_wp_stopping_point_xy(Vector3f& stopping_point) {}
	bool reached_wp_destination() const { return waypopint_destination_is_reached; }
	const Vector3f &get_wp_destination() const {Vector3f toto; return toto; }
	void wp_and_spline_init() {};
	void set_wp_destination(const Vector3f& destination) {};
	void update_wpnav() {};
	int32_t get_roll() const { return 0; }
	int32_t get_pitch() const { return 0; };
};//AC_WPNav

#endif /* AC_WPNAV_STUB_H_ */
