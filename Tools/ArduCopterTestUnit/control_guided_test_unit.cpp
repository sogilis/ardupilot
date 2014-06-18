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

#define AUTO_YAW_HOLD                   0

// Function Stubs

uint32_t millis() {return 0;}
uint32_t micros() {return 0;}
void fence_send_mavlink_status(mavlink_channel_t channel) {}
void get_angle_targets_for_reporting(Vector3f targets) {}
uint16_t comm_get_available(mavlink_channel_t chan) {return 3;}
uint8_t comm_receive_ch(mavlink_channel_t chan) {return 3;}
uint16_t comm_get_txspace(mavlink_channel_t chan) {return 32;}
bool comm_is_idle(mavlink_channel_t chan) {return true;}
void run_cli(UARTDriver* port){}
uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid){return 0;}
void init_barometer(bool full_calibration) { }
void trim_radio() { }
void pre_arm_checks (bool set) {}
bool arm_checks (bool set) {return true;}
void init_arm_motors() { }
uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec) {return 7;}
void init_disarm_motors() { }
uint8_t mavlink_compassmot(mavlink_channel_t chan) {return 3;}
void check_usb_mux(void) {} ;
// Stubbed method
static void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, uint8_t relative_angle); // control_auto
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd) {};
static void auto_rtl_start() {};
static float max (float a, float b) {return a;}
Vector3f pv_location_to_vector(const Location& loc) {Vector3f test; return test;}
static void auto_wp_start(const Vector3f& destination) {}
static void auto_land_start() {}
static void auto_land_start(const Vector3f& destination) {} ;
static void auto_circle_start() {}
static void auto_circle_movetoedge_start() {}
static void Log_Write_Event(uint8_t id) {}
static void parachute_release() {}
static void gcs_send_text_fmt(const prog_char_t *fmt, ...) {};
int32_t wrap_360_cd(int32_t error) {};
int32_t wrap_180_cd(int32_t error) {};
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {}
static void calc_wp_distance() {}
void set_auto_yaw_mode(uint8_t yaw_mode) {}
static void init_home() {}
void set_home_is_set(bool b) {}
uint8_t get_default_auto_yaw_mode(bool rtl) {}
static void Log_Write_Camera() {}
static void auto_spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_destination) {}
static bool GPS_ok() {}
static float get_pilot_desired_yaw_rate(int16_t stick_angle) {}
float get_auto_heading(void) {}
void auto_takeoff_start_factor(float final_alt){};

// Variables Stubs

static AP_Motors motors;
static AP_InertialSensor ins;
static AP_AHRS ahrs;
static Parameters g;// = {true, true};
static AP_GPS gps;
static AP_COMPASS compass;
static GPS_Glitch gps_glitch;
static AP_BattMonitor battery;
static AP_Scheduler scheduler;
static AC_PosControl pos_control;
static SITL sitl;
static AP_HAL hal;
static AP_Baro barometer;
static AP_Mission mission;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];
static UARTDriver* _port;
static DataFlash_Class DataFlash;
static AC_Fence fence;
static AP_ServoRelayEvents ServoRelayEvents;
static AP_Camera camera;
static AP_Mount camera_mount;
static AP_Rally rally;
static RallyLocation rally_point;
static AP_Notify notify;
static AC_WPNav wp_nav;
static uint8_t command_ack_counter;
static Location current_loc;
static Location home;
static uint8_t receiver_rssi = 1;
static int16_t climb_rate;
static int16_t sonar_alt;
static uint32_t _cli_timeout;
static uint8_t  crlf_count;
static int16_t pmTest1;
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
struct AP_Notify::notify_type AP_Notify::flags;
static int8_t control_mode = STABILIZE;
static int32_t  wp_bearing;
static uint32_t wp_distance;
static AP_InertialNav inertial_nav(ahrs, barometer, gps_glitch);
static uint32_t loiter_time;
static uint16_t loiter_time_max;
static uint8_t land_state;
static AC_Circle circle_nav(inertial_nav, ahrs, pos_control);
static AP_Parachute parachute;
uint16_t checksum; /// sent at end of packet
uint8_t magic;   ///< protocol magic marker
uint8_t len;     ///< Length of payload
uint8_t seq;     ///< Sequence of packet
uint8_t sysid;   ///< ID of message sender system/aircraft
uint8_t compid;  ///< ID of the message sender component
uint8_t msgid;   ///< ID of message in payload
uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
static AutoMode auto_mode;   // controls which auto controller is run
RTLState rtl_state;
static uint32_t condition_start;
static int32_t condition_value;
static int32_t yaw_look_at_heading;
static int16_t yaw_look_at_heading_slew;
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
	bool auto_takeoff_run_has_been_called;
	bool auto_takeoff_start_has_been_called;
	float altitude_setup;
	uint8_t mode;
};//Take_Off_Stub

static Take_Off_Stub take_Off_Stub;

static void auto_takeoff_run() {
	take_Off_Stub.auto_takeoff_run_has_been_called = true;
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



uint8_t result [11];
int result_index = 0;


void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
	result [result_index] = ch;
	result_index++;
}//comm_send_ch


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
	REQUIRE (take_Off_Stub.auto_takeoff_run_has_been_called);
}

