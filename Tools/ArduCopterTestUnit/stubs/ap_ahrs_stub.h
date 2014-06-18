#ifndef AP_AHRS_STUB_H_
#define AP_AHRS_STUB_H_

#include "util.h"

class AP_AHRS {
	Vector3f gyro_drift;
    struct Location _home;
public:
	float roll;
	float pitch;
	float yaw;
	int32_t yaw_sensor;

	bool use_compass () {return true;}
	bool healthy () {return true;}
    const Vector3f &get_gyro_drift() const {return gyro_drift;}
    float get_error_rp(void) {return 0.0;}
    float get_error_yaw(void) {return 0.0;}
    void  set_trim(Vector3f new_trim) {}
    void set_home(const Location &loc) {}

    const struct Location &get_home(void) const {return _home;}



};//AP_AHRS

#endif /* AP_AHRS_STUB_H_ */
