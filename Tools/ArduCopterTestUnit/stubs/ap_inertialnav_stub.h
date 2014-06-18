/*
 * ap_inertialnav.h
 *
 *  Created on: 17 juin 2014
 *      Author: valentin
 */

#ifndef AP_INERTIALNAV_STUB_H_
#define AP_INERTIALNAV_STUB_H_

class AP_InertialNav
{

public:

	// Constructor
	    AP_InertialNav(AP_AHRS &ahrs, AP_Baro &baro, GPS_Glitch& gps_glitch )
	    {

	    }

	    virtual const Vector3f&    get_position() const { return _position; }
	    virtual float       get_altitude() const { return _position.z; }

protected:

	    Vector3f _position;

};


#endif /* AP_INERTIALNAV_STUB_H_ */
