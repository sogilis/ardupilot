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

// Guided modes
enum GuidedMode {
    Guided_TakeOff,
    Guided_WP
};


#define PARAM1_OFFSET 0
#define PARAM2_OFFSET 4
#define PARAM3_OFFSET 8
#define PARAM4_OFFSET 12
#define PARAM5_OFFSET 16
#define PARAM6_OFFSET 20
#define PARAM7_OFFSET 24

#define AUTO_YAW_HOLD  0

// Stubbed method
static float get_pilot_desired_yaw_rate(int16_t stick_angle) {}
static bool GPS_ok() {}
void set_auto_yaw_mode(uint8_t yaw_mode) {}
uint8_t get_default_auto_yaw_mode(bool rtl) {}
float get_auto_heading(void) {}
void auto_takeoff_start_factor(float final_alt){};
void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {}

// Variables Stubs
static AP_AHRS ahrs;
static Parameters g;// = {true, true};
static GPS_Glitch gps_glitch;
static AC_PosControl pos_control;
static AC_WPNav wp_nav;
static int8_t control_mode = STABILIZE;
static AP_Baro barometer;
static AP_InertialNav inertial_nav(ahrs, barometer, gps_glitch);
static AutoMode auto_mode;   // controls which auto controller is run
RTLState rtl_state;
AC_AttitudeControl attitude_control;
static uint8_t auto_yaw_mode;
static Vector3f roi_WP;
bool rtl_state_complete;
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
	bool auto_takeoff_run_factor_has_been_called;
	bool auto_takeoff_start_has_been_called;
	float altitude_setup;
	uint8_t mode;
};//Take_Off_Stub

static Take_Off_Stub take_Off_Stub;

static void auto_takeoff_run_factor() {
	take_Off_Stub.auto_takeoff_run_factor_has_been_called = true;
}

static void auto_takeoff_start(float final_alt) {
	take_Off_Stub.auto_takeoff_start_has_been_called = true;
	take_Off_Stub.altitude_setup = final_alt;
}

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

TEST_CASE("Guided Take Off Setup", "Take Off - Setup in guided mode") {
	guided_mode = Guided_WP;
	guided_takeoff_start(3.0);
	REQUIRE (guided_mode == Guided_TakeOff);

}

TEST_CASE("Guided Take Off Run", "Take Off - Run in guided mode") {
	guided_mode = Guided_TakeOff;
	guided_run ();
	REQUIRE (take_Off_Stub.auto_takeoff_run_factor_has_been_called);
}

TEST_CASE("Guided Take Off Stop", "Take Off - End") {
	guided_mode = Guided_TakeOff;
	wp_nav.waypopint_destination_is_reached = false;
	guided_run ();
	REQUIRE (guided_mode == Guided_TakeOff);
	wp_nav.waypopint_destination_is_reached = true;
	guided_run ();
	REQUIRE (guided_mode == Guided_WP);
}

TEST_CASE("Guided Point Rejected During Take Off", "Take Off - End") {
	guided_mode = Guided_TakeOff;
	control_mode = GUIDED;
	Vector3f wp_to_test;
	wp_nav.targetPos.x = 0.0;
	wp_nav.targetPos.y = 0.0;
	wp_nav.targetPos.z = 0.0;
	wp_to_test.x = 1.0;
	wp_to_test.y = 1.0;
	wp_to_test.z = 1.0;
	guided_set_destination (wp_to_test);
	REQUIRE (wp_nav.targetPos.x == 0.0);
	REQUIRE (wp_nav.targetPos.y == 0.0);
	REQUIRE (wp_nav.targetPos.z == 0.0);
}

