/*
 * ap_mount_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef AP_MOUNT_STUB_H_
#define AP_MOUNT_STUB_H_

class AP_Mount
{
public:

    enum MountType {
        k_unknown = 0,                  ///< unknown type
        k_pan_tilt = 1,                 ///< yaw-pitch
        k_tilt_roll = 2,                ///< pitch-roll
        k_pan_tilt_roll = 3,            ///< yaw-pitch-roll
    };

    typedef enum MAV_MOUNT_MODE
    {
    	MAV_MOUNT_MODE_RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | */
    	MAV_MOUNT_MODE_NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
    	MAV_MOUNT_MODE_MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
    	MAV_MOUNT_MODE_RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
    	MAV_MOUNT_MODE_GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
    	MAV_MOUNT_MODE_ENUM_END=5, /*  | */
    } MAV_MOUNT_MODE;

	void configure_msg(mavlink_message_t* msg) {}
	void control_msg(mavlink_message_t* msg) {}
    void status_msg(mavlink_message_t* msg) {}
    void control_cmd() {}
    void configure_cmd() {}
    void set_roi_cmd(const struct Location *target_loc) {}
    MAV_MOUNT_MODE get_mode() const { return MAV_MOUNT_MODE_RETRACT; }
    enum MountType          get_mount_type() {
            return k_tilt_roll;
        }
    void                    set_mode_to_default() { ; }

};

#endif /* AP_MOUNT_STUB_H_ */
