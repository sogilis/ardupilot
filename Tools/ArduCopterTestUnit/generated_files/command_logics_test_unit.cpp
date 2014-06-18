#line 1 "commands_logic_test_unit.cpp"
/*
 * stub.cpp
 *
 *  Created on: 22 mai 2014
 *      Author: valentin
 */

#define CONFIG_HAL_BOARD HAL_BOARD_AVR_SITL
#define FRAME_CONFIG QUAD_FRAME

#include "commands_logic_test_unit.h"
#line 1 "autogenerated"
  uint32_t millis() ;
 uint32_t micros() ;
 void fence_send_mavlink_status(mavlink_channel_t channel) ;
 void get_angle_targets_for_reporting(Vector3f targets) ;
 uint16_t comm_get_available(mavlink_channel_t chan) ;
 uint8_t comm_receive_ch(mavlink_channel_t chan) ;
 uint16_t comm_get_txspace(mavlink_channel_t chan) ;
 bool comm_is_idle(mavlink_channel_t chan) ;
 void run_cli(UARTDriver* port);
 uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid);
 void init_barometer(bool full_calibration) ;
 void trim_radio() ;
 void pre_arm_checks (bool set) ;
 bool arm_checks (bool set) ;
 void init_arm_motors() ;
 uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec) ;
 void init_disarm_motors() ;
 uint8_t mavlink_compassmot(mavlink_channel_t chan) ;
 void check_usb_mux(void) ;
 static void auto_rtl_start() ;
 static void auto_takeoff_start(float final_alt) ;
 static float max (float a, float b) ;
 Vector3f pv_location_to_vector(const Location& loc) ;
 static void auto_wp_start(const Vector3f& destination) ;
 static void auto_land_start() ;
 static void auto_land_start(const Vector3f& destination) ;
 static void auto_circle_start() ;
 static void auto_circle_movetoedge_start() ;
 static void Log_Write_Event(uint8_t id) ;
 static void parachute_release() ;
 int32_t wrap_360_cd(int32_t error) ;
 int32_t wrap_180_cd(int32_t error) ;
 int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) ;
 static void calc_wp_distance() ;
 void set_auto_yaw_mode(uint8_t yaw_mode) ;
 static void guided_set_destination(const Vector3f& destination) ;
 static void init_home() ;
 void set_home_is_set(bool b) ;
 uint8_t get_default_auto_yaw_mode(bool rtl) ;
 static void Log_Write_Camera() ;
   void comm_send_ch(mavlink_channel_t chan, uint8_t ch) ;
   static void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, uint8_t relative_angle) ;
  bool set_mode(uint8_t mode) ;
  	void reset(void) ;
  	uint8_t get_result(int index) ;
 static void exit_mission() ;
 static void do_RTL(void) ;
 static bool verify_takeoff() ;
 static bool verify_land() ;
  static bool verify_loiter_unlimited() ;
 static bool verify_loiter_time() ;
 static bool verify_RTL() ;
  static bool verify_wait_delay() ;
  static bool verify_change_alt() ;
  static bool verify_within_distance() ;
 static bool verify_yaw() ;
 static void do_take_picture() ;
#line 12 "commands_logic_test_unit.cpp"
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
static void auto_takeoff_start(float final_alt) {} ;
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
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {};
static void calc_wp_distance() {} ;
void set_auto_yaw_mode(uint8_t yaw_mode) {} ;
static void guided_set_destination(const Vector3f& destination) {} ;
static void init_home() {} ;
void set_home_is_set(bool b) {} ;
uint8_t get_default_auto_yaw_mode(bool rtl) {} ;
static void Log_Write_Camera() {} ;
static void auto_spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_destination) {}
static void do_takeoff(const AP_Mission::Mission_Command& cmd);
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

class Control_Auto_Stub {
public:
	float   angle_deg;      // target angle in degrees (0=north, 90=east)
	float   turn_rate_dps;  // turn rate in degrees / second (0=use default)
	uint8_t relative_angle; // 0 = absolute angle, 1 = relative angle
};//Control_Auto_Stub
static Control_Auto_Stub control_auto_stub;

class Take_Off_Stub {
public:
	bool has_been_called;
	AP_Mission::Mission_Command cmd;
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

bool set_mode(uint8_t mode) {
	take_Off_Stub.mode = mode;
	return true;
}//set_mode


class Fixture {

	mavlink_message_t msg;

	void reset(void) {
		result_index = 0;
		int i = 0;
		for (i = 0; i < 11; i++) {
			result[i] = 0;
		}
	}

	uint8_t get_result(int index) {
		return result[index];
	}

public:

};

TEST_CASE("Speed Invalid Case", "COMMAND_LONG | DO_CHANGE_SPEED") {
	REQUIRE (true);
}

#line 1 "../../ArduCopter/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// forward declarations to make compiler happy
static void do_nav_wp(const AP_Mission::Mission_Command& cmd);
static void do_land(const AP_Mission::Mission_Command& cmd);
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
static void do_circle(const AP_Mission::Mission_Command& cmd);
static void do_loiter_time(const AP_Mission::Mission_Command& cmd);
static void do_spline_wp(const AP_Mission::Mission_Command& cmd);
static void do_wait_delay(const AP_Mission::Mission_Command& cmd);
static void do_within_distance(const AP_Mission::Mission_Command& cmd);
static void do_change_alt(const AP_Mission::Mission_Command& cmd);
static void do_yaw(const AP_Mission::Mission_Command& cmd);
static void do_change_speed(const AP_Mission::Mission_Command& cmd);
static void do_set_home(const AP_Mission::Mission_Command& cmd);
static void do_roi(const AP_Mission::Mission_Command& cmd);
#if PARACHUTE == ENABLED
static void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
static bool verify_circle(const AP_Mission::Mission_Command& cmd);
static bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
static void auto_spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_spline_destination);

// start_command - this function will be called when the ap_mission lib wishes to start a new command
static bool start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (g.log_bitmask & MASK_LOG_CMD) {
        Log_Write_Cmd(cmd);
    }

    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_spline_wp(cmd);
        break;

    //
    // conditional commands
    //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:             // 113
        do_change_alt(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

    ///
    /// do commands
    ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_SERVO:
        ServoRelayEvents.do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
        break;
        
    case MAV_CMD_DO_SET_RELAY:
        ServoRelayEvents.do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
        break;
        
    case MAV_CMD_DO_REPEAT_SERVO:
        ServoRelayEvents.do_repeat_servo(cmd.content.repeat_servo.channel, cmd.content.repeat_servo.pwm,
                                         cmd.content.repeat_servo.repeat_count, cmd.content.repeat_servo.cycle_time * 1000.0f);
        break;
        
    case MAV_CMD_DO_REPEAT_RELAY:
        ServoRelayEvents.do_repeat_relay(cmd.content.repeat_relay.num, cmd.content.repeat_relay.repeat_count,
                                         cmd.content.repeat_relay.cycle_time * 1000.0f);
        break;

    case MAV_CMD_DO_SET_ROI:                // 201
        // point the copter and camera at a region of interest (ROI)
        do_roi(cmd);
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_take_picture();
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
        break;
#endif

#if MOUNT == ENABLED
    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:                          // Mission command to configure or release parachute
        do_parachute(cmd);
        break;
#endif

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }

    // always return success
    return true;
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_command - this will be called repeatedly by ap_mission lib to ensure the active commands are progressing
//  should return true once the active navigation command completes successfully
//  called at 10hz or higher
static bool verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch(cmd.id) {

    //
    // navigation commands
    //
    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        return verify_spline_wp(cmd);
        break;

    ///
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();
        break;

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // assume parachute was released successfully
        return true;
        break;
#endif

    default:
        // return true if we do not recognise the command so that we move on to the next command
        return true;
        break;
    }
}

// exit_mission - function that is called once the mission completes
static void exit_mission()
{
    // if we are not on the ground switch to loiter or land
    if(!ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!set_mode(LOITER)) {
            set_mode(LAND);
        }
    }else{
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (g.rc_3.control_in == 0 || failsafe.radio) {
            init_disarm_motors();
        }
#else
        // if we've landed it's safe to disarm
        init_disarm_motors();
#endif
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // start rtl in auto flight mode
    auto_rtl_start();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
static void do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    float takeoff_alt = cmd.content.location.alt;
    takeoff_alt = max(takeoff_alt,current_loc.alt);
    takeoff_alt = max(takeoff_alt,100.0f);
    auto_takeoff_start(takeoff_alt);
}

// do_nav_wp - initiate move to next waypoint
static void do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    Vector3f local_pos = pv_location_to_vector(cmd.content.location);

    // set target altitude to current altitude if not provided
    if (cmd.content.location.alt == 0) {
        local_pos.z = curr_pos.z;
    }

    // set lat/lon position to current position if not provided
    if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        local_pos.x = curr_pos.x;
        local_pos.y = curr_pos.y;
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // Set wp navigation target
    auto_wp_start(local_pos);
    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0 ) {
        wp_nav.set_fast_waypoint(true);
    }
}

// do_land - initiate landing procedure
static void do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        land_state = LAND_STATE_FLY_TO_LOCATION;

        // calculate and set desired location above landing target
        Vector3f pos = pv_location_to_vector(cmd.content.location);
        pos.z = current_loc.alt;
        auto_wp_start(pos);
    }else{
        // set landing state
        land_state = LAND_STATE_DESCENDING;

        // initialise landing controller
        auto_land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);
}

// do_circle - initiate moving in a circle
static void do_circle(const AP_Mission::Mission_Command& cmd)
{
    Vector3f curr_pos = inertial_nav.get_position();
    Vector3f circle_center = pv_location_to_vector(cmd.content.location);
    bool move_to_edge_required = false;

    // set target altitude if not provided
    if (cmd.content.location.alt == 0) {
        circle_center.z = curr_pos.z;
    } else {
        move_to_edge_required = true;
    }

    // set lat/lon position if not provided
    // To-Do: use previous command's destination if it was a straight line or spline waypoint command
    if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        circle_center.x = curr_pos.x;
        circle_center.y = curr_pos.y;
    } else {
        move_to_edge_required = true;
    }

    // set circle controller's center
    circle_nav.set_center(circle_center);

    // check if we need to move to edge of circle
    if (move_to_edge_required) {
        // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
        auto_circle_movetoedge_start();
    } else {
        // start circling
        auto_circle_start();
    }
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
static void do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_spline_wp - initiate move to next waypoint
static void do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    Vector3f local_pos = pv_location_to_vector(cmd.content.location);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // determine segment start and end type
    bool stopped_at_start = true;
    AC_WPNav::spline_segment_end_type seg_end_type = AC_WPNav::SEGMENT_END_STOP;
    AP_Mission::Mission_Command temp_cmd;
    Vector3f next_destination;      // end of next segment

    // if previous command was a wp_nav command with no delay set stopped_at_start to false
    // To-Do: move processing of delay into wp-nav controller to allow it to determine the stopped_at_start value itself?
    uint16_t prev_cmd_idx = mission.get_prev_nav_cmd_index();
    if (prev_cmd_idx != AP_MISSION_CMD_INDEX_NONE) {
        if (mission.read_cmd_from_storage(prev_cmd_idx, temp_cmd)) {
            if ((temp_cmd.id == MAV_CMD_NAV_WAYPOINT || temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) && temp_cmd.p1 == 0) {
                stopped_at_start = false;
            }
        }
    }

    // if there is no delay at the end of this segment get next nav command
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        // if the next nav command is a waypoint set end type to spline or straight
        if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_STRAIGHT;
            next_destination = pv_location_to_vector(temp_cmd.content.location);
        }else if (temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_SPLINE;
            next_destination = pv_location_to_vector(temp_cmd.content.location);
        }
    }

    // set spline navigation target
    auto_spline_start(local_pos, stopped_at_start, seg_end_type, next_destination);
}

#if PARACHUTE == ENABLED
// do_parachute - configure or release parachute
static void do_parachute(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.p1) {
        case PARACHUTE_DISABLE:
            parachute.enabled(false);
            Log_Write_Event(DATA_PARACHUTE_DISABLED);
            break;
        case PARACHUTE_ENABLE:
            parachute.enabled(true);
            Log_Write_Event(DATA_PARACHUTE_ENABLED);
            break;
        case PARACHUTE_RELEASE:
            parachute_release();
            break;
        default:
            // do nothing
            break;
    }
}
#endif

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
static bool verify_takeoff()
{
    // have we reached our target altitude?
    return wp_nav.reached_wp_destination();
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    bool retval = false;

    switch( land_state ) {
        case LAND_STATE_FLY_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = wp_nav.get_wp_destination();

                // initialise landing controller
                auto_land_start(dest);

                // advance to next state
                land_state = LAND_STATE_DESCENDING;
            }
            break;

        case LAND_STATE_DESCENDING:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

// verify_nav_wp - check if we have reached the next way point
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}

static bool verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
static bool verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
static bool verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (wp_nav.reached_wp_destination()) {
            Vector3f curr_pos = inertial_nav.get_position();
            Vector3f circle_center = pv_location_to_vector(cmd.content.location);

            // set target altitude if not provided
            if (circle_center.z == 0) {
                circle_center.z = curr_pos.z;
            }

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.x = curr_pos.x;
                circle_center.y = curr_pos.y;
            }

            // start circling
            auto_circle_start();
        }
        return false;
    }

    // check if we have completed circling
    return fabsf(circle_nav.get_angle_total()/(2*M_PI)) >= cmd.p1;
}

// externs to remove compiler warning
extern bool rtl_state_complete;

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    return (rtl_state_complete && (rtl_state == FinalDescent || rtl_state == Land));
}

// verify_spline_wp - check if we have reached the next way point using spline
static bool verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

static void do_change_alt(const AP_Mission::Mission_Command& cmd)
{
    // adjust target appropriately for each nav mode
    if (control_mode == AUTO) {
        switch (auto_mode) {
        case Auto_TakeOff:
            // To-Do: adjust waypoint target altitude to new provided altitude
            break;
        case Auto_WP:
        case Auto_Spline:
            // To-Do; reset origin to current location + stopping distance at new altitude
            break;
        case Auto_Land:
        case Auto_RTL:
            // ignore altitude
            break;
        case Auto_CircleMoveToEdge:
        case Auto_Circle:
            // move circle altitude up to target (we will need to store this target in circle class)
            break;
        }
    }
    // To-Do: store desired altitude in a variable so that it can be verified later
}

static void do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

static void do_yaw(const AP_Mission::Mission_Command& cmd)
{
	set_auto_yaw_look_at_heading(
		cmd.content.yaw.angle_deg,
		cmd.content.yaw.turn_rate_dps,
		cmd.content.yaw.relative_angle);
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

static bool verify_change_alt()
{
    // To-Do: use recorded target altitude to verify we have reached the target
    return true;
}

static bool verify_within_distance()
{
    // update distance calculation
    calc_wp_distance();
    if (wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
static bool verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    // check if we are within 2 degrees of the target heading
    if (labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200) {
        return true;
    }else{
        return false;
    }
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

// do_guided - start guided mode
static bool do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (control_mode != GUIDED) {
        return false;
    }

    // set wp_nav's destination
    Vector3f pos = pv_location_to_vector(cmd.content.location);
    guided_set_destination(pos);
    return true;
}

static void do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        wp_nav.set_speed_xy(cmd.content.speed.target_ms * 100.0f);
    }
}

static void do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if(cmd.p1 == 1) {
        init_home();
    } else {
        Location loc = cmd.content.location;
        ahrs.set_home(loc);
        set_home_is_set(true);
    }
}

// do_roi - starts actions required by MAV_CMD_NAV_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//          Note: the ROI should already be in the command_nav_queue global variable
//	TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
static void do_roi(const AP_Mission::Mission_Command& cmd)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (auto_yaw_mode == AUTO_YAW_ROI && (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0)) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
#if MOUNT == ENABLED
        // switch off the camera tracking if enabled
        if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
            camera_mount.set_mode_to_default();
        }
#endif  // MOUNT == ENABLED
    }else{
#if MOUNT == ENABLED
        // check if mount type requires us to rotate the quad
        if(camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll) {
            roi_WP = pv_location_to_vector(cmd.content.location);
            set_auto_yaw_mode(AUTO_YAW_ROI);
        }
        // send the command to the camera mount
        camera_mount.set_roi_cmd(&cmd.content.location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //		0: do nothing
        //		1: point at next waypoint
        //		2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //		3: point at a location given by alt, lon, lat parameters
        //		4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the quad at the location
        roi_WP = pv_location_to_vector(cmd.content.location);
        set_auto_yaw_mode(AUTO_YAW_ROI);
#endif  // MOUNT == ENABLED
    }
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    if (g.log_bitmask & MASK_LOG_CAMERA) {
        Log_Write_Camera();
    }
#endif
}
