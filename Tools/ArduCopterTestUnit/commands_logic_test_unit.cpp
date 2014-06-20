/*
 * stub.cpp
 *
 *  Created on: 22 mai 2014
 *      Author: valentin
 */

#define CONFIG_HAL_BOARD HAL_BOARD_AVR_SITL
#define FRAME_CONFIG QUAD_FRAME

#include "commands_logic_test_unit.h"
mavlink_system_t mavlink_system = {7,1,0,0,0,0};
#include <mavlink.h>
#include <stdio.h>
#include <math.h>
#include "stubs/gcs_mavlink_stub.h"
#include "catch.hpp"

enum AutoMode {
    Auto_TakeOff,
    Auto_WP,
    Auto_Land,
    Auto_RTL,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_Spline
};

// RTL states
enum RTLState {
    InitialClimb,
    ReturnHome,
    LoiterAtHome,
    FinalDescent,
    Land
};


#define PARAM1_OFFSET 0
#define PARAM2_OFFSET 4
#define PARAM3_OFFSET 8
#define PARAM4_OFFSET 12
#define PARAM5_OFFSET 16
#define PARAM6_OFFSET 20
#define PARAM7_OFFSET 24

// Function Stubs

uint32_t millis() {return 0;}
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd) {};
static void gcs_send_text_fmt(const prog_char_t *fmt, ...) {};
void init_disarm_motors() { }
static void auto_rtl_start() {};
static float max (float a, float b) {return a;}
static void auto_takeoff_start(float final_alt) {} ;
Vector3f pv_location_to_vector(const Location& loc) {Vector3f test; return test;}
static void auto_wp_start(const Vector3f& destination) {}
static void auto_circle_movetoedge_start() {}
static void auto_circle_start() {}
static void auto_spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_destination) {}
static void Log_Write_Event(uint8_t id) {}
static void parachute_release() {}
static void auto_land_start(const Vector3f& destination) {} ;
static void auto_land_start() {} ;
static void calc_wp_distance() {} ;
void set_auto_yaw_mode(uint8_t yaw_mode) {} ;
int32_t wrap_180_cd(int32_t error) {};
static void guided_set_destination(const Vector3f& destination) {} ;
static void init_home() {} ;
void set_home_is_set(bool b) {} ;
uint8_t get_default_auto_yaw_mode(bool rtl) {} ;
static void Log_Write_Camera() {} ;
void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {}//comm_send_ch

// Variables Stubs

static Parameters g;// = {true, true};
static AP_ServoRelayEvents ServoRelayEvents;
static AP_Baro barometer;
static AP_Camera camera;
static AP_Mount camera_mount;
static Location current_loc;
static GPS_Glitch gps_glitch;
static AP_AHRS ahrs;
static AP_InertialNav inertial_nav(ahrs, barometer, gps_glitch);
static uint16_t loiter_time_max;
static uint32_t loiter_time;
static AC_WPNav wp_nav;
static uint8_t land_state;
static AP_Parachute parachute;
static uint32_t condition_start;
static Vector3f roi_WP;
static uint8_t auto_yaw_mode;
static int8_t control_mode = STABILIZE;
static int32_t yaw_look_at_heading;
static int32_t condition_value;
static AutoMode auto_mode;   // controls which auto controller is run
static uint32_t wp_distance;
RTLState rtl_state;
static AC_PosControl pos_control;
static AC_Circle circle_nav(inertial_nav, ahrs, pos_control);
static AP_Mission mission;
bool rtl_state_complete;

// Guided modes
enum GuidedMode {
    Guided_TakeOff,
    Guided_WP
};

static GuidedMode guided_mode;


class Control_Auto_Stub {
public:
	float   angle_deg;      // target angle in degrees (0=north, 90=east)
	float   turn_rate_dps;  // turn rate in degrees / second (0=use default)
	uint8_t relative_angle; // 0 = absolute angle, 1 = relative angle
};//Control_Auto_Stub
static Control_Auto_Stub control_auto_stub;

class Take_Off_Stub {
public:
	bool guided_takeoff_start_has_been_called;
	float altitude;
	uint8_t mode;
};//Take_Off_Stub
static Take_Off_Stub take_Off_Stub;


static union {
    struct {
        uint8_t home_is_set         : 1; // 0
        uint8_t simple_mode         : 2; // 1,2 // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
        uint8_t pre_arm_rc_check    : 1; // 3   // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check       : 1; // 4   // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
        uint8_t logging_started     : 1; // 6   // true if dataflash logging has started
        uint8_t land_complete       : 1; // 7   // true if we have detected a landing
        uint8_t new_radio_frame     : 1; // 8       // Set true if we have new PWM data to act on from the Radio
        uint8_t CH7_flag            : 2; // 9,10   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag            : 2; // 11,12   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t usb_connected       : 1; // 13      // true if APM is powered from USB connection
        uint8_t rc_receiver_present : 1; // 14  // true if we have an rc receiver present (i.e. if we've ever received an update
        uint8_t compass_mot         : 1; // 15  // true if we are currently performing compassmot calibration
        uint8_t motor_test          : 1; // 16  // true if we are currently performing the motors test
    };
    uint32_t value;
} ap;

static struct {
    uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
    uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
    uint8_t battery             : 1; // 2   // A status flag for the battery failsafe
    uint8_t gps                 : 1; // 3   // A status flag for the gps failsafe
    uint8_t gcs                 : 1; // 4   // A status flag for the ground station failsafe
    int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value
    uint32_t last_heartbeat_ms;      // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
} failsafe;

static void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, uint8_t relative_angle) {
	control_auto_stub.angle_deg      = angle_deg;
	control_auto_stub.turn_rate_dps  = turn_rate_dps;
	control_auto_stub.relative_angle = relative_angle;
}//set_auto_yaw_look_at_heading

bool set_mode(uint8_t mode) {
	take_Off_Stub.mode = mode;
	return true;
}//set_mode


static void guided_takeoff_start(float alt) {
	take_Off_Stub.guided_takeoff_start_has_been_called = true;
	take_Off_Stub.altitude = alt;
}



class Fixture {

public:

	void exercize (float altitude){
		do_takeoff_guided (altitude);
	}

};

TEST_CASE("Speed Invalid Case", "COMMAND_LONG | DO_CHANGE_SPEED") {
	Fixture fixture;
	guided_mode = Guided_WP;
	fixture.exercize(3.0);
	REQUIRE (take_Off_Stub.guided_takeoff_start_has_been_called);
	REQUIRE (take_Off_Stub.altitude == 3.0);
}

