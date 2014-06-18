/*
 * attitude_control_helo_stub.h
 *
 *  Created on: 17 juin 2014
 *      Author: valentin
 */

#ifndef AC_ATTITUDE_CONTROL_STUB_H_
#define AC_ATTITUDE_CONTROL_STUB_H_


class AC_AttitudeControl {

public:
	 const Vector3f& angle_ef_targets() const { Vector3f toto; return toto; }
	 void relax_bf_rate_controller() {}
	 void set_yaw_target_to_current_heading() {}
	 void set_throttle_out(int16_t throttle_pwm, bool apply_angle_boost) {}
	 void angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef) {}
	 void angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw) {}

};


#endif /* AC_ATTITUDE_CONTROL_HELI_STUB_H_ */
