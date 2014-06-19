/*
 * stub.cpp
 *
 *  Created on: 22 mai 2014
 *      Author: valentin
 */

#define CONFIG_HAL_BOARD HAL_BOARD_AVR_SITL
#define FRAME_CONFIG QUAD_FRAME

#include "gcs_mavlink_test_unit.h"
mavlink_system_t mavlink_system = {7,1,0,0,0,0};
#include <mavlink.h>
#include <stdio.h>
#include "stubs/gcs_mavlink_stub.h"
#include "catch.hpp"


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
static bool do_guided(const AP_Mission::Mission_Command& cmd) {return true;}
// Stubbed method
static void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, uint8_t relative_angle); // control_auto
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
uint16_t checksum; /// sent at end of packet
uint8_t magic;   ///< protocol magic marker
uint8_t len;     ///< Length of payload
uint8_t seq;     ///< Sequence of packet
uint8_t sysid;   ///< ID of message sender system/aircraft
uint8_t compid;  ///< ID of the message sender component
uint8_t msgid;   ///< ID of message in payload
uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];

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

static void do_takeoff_guided(float altitude) {
	take_Off_Stub.has_been_called = true;
	take_Off_Stub.altitude = altitude;
}//do_takeoff

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
	GCS_MAVLINK       sut;
	uint8_t 		  tab[4];

	void setup_speed_test (float initial_speed_in_cm_per_s) {
		reset();
		wp_nav.speed_in_cms = initial_speed_in_cm_per_s;
		uint16_t unused_checksum = 0;
		uint8_t  unused_protocol_magic = 0;
		uint8_t  unused_payload_lenght = 0;
		uint8_t  unused_sequence = 0;
		uint8_t  unused_sysid = 0;
		uint8_t  unused_compid = 0;
		uint8_t  command_long_msg_ig = 0x4C;
		msg =
		{
				unused_checksum,
				unused_protocol_magic,
				unused_payload_lenght,
				unused_sequence,
				unused_sysid,
				unused_compid,
				command_long_msg_ig, {}
		};
		uint8_t* payload_ptr = (uint8_t*) msg.payload64;
		payload_ptr [28] = 0xB2; 						// Command = 178 for DO_CHANGE_SPEED
		payload_ptr [29] = 0x00;
		payload_ptr [4]  = tab [0]; // Param 2 : Speed to set
		payload_ptr [5]  = tab [1];
		payload_ptr [6]  = tab [2];
		payload_ptr [7]  = tab [3];
	}//setup_speed_test

	void setup_yaw_test (float param1, float param2, float param4) {
		reset();
		uint16_t unused_checksum = 0;
		uint8_t  unused_protocol_magic = 0;
		uint8_t  unused_payload_lenght = 0;
		uint8_t  unused_sequence = 0;
		uint8_t  unused_sysid = 0;
		uint8_t  unused_compid = 0;
		uint8_t  command_long_msg_ig = 0x4C;
		msg =
		{
				unused_checksum,
				unused_protocol_magic,
				unused_payload_lenght,
				unused_sequence,
				unused_sysid,
				unused_compid,
				command_long_msg_ig, {}
		};
		uint8_t* payload_ptr = (uint8_t*) msg.payload64;
		float* param;
		param = (float*)&payload_ptr[PARAM1_OFFSET];
		*param = param1;
		param = (float*)&payload_ptr[PARAM2_OFFSET];
		*param = param2;
		param = (float*)&payload_ptr[PARAM4_OFFSET];
		*param = param4;
		payload_ptr [28] = 0x73;    // Command = 115 for CONDITION_YAW
		payload_ptr [29] = 0x00;
		// Parameters

	}//setup_yaw_test

	void setup_take_off_test (float param7) {
		reset();
		uint16_t unused_checksum = 0;
		uint8_t  unused_protocol_magic = 0;
		uint8_t  unused_payload_lenght = 0;
		uint8_t  unused_sequence = 0;
		uint8_t  unused_sysid = 0;
		uint8_t  unused_compid = 0;
		uint8_t  command_long_msg_ig = 0x4C;
		msg =
		{
				unused_checksum,
				unused_protocol_magic,
				unused_payload_lenght,
				unused_sequence,
				unused_sysid,
				unused_compid,
				command_long_msg_ig, {}
		};
		uint8_t* payload_ptr = (uint8_t*) msg.payload64;
		float* param;
		param = (float*)&payload_ptr[PARAM7_OFFSET];
		*param = param7;
		payload_ptr [28] = 0x16;    // Command = 22  for TAKEOFF
		payload_ptr [29] = 0x00;
		// Parameters

	}//setup_take_off_test

	void exercize (void) {
		sut.handleMessage(&msg);
	}//exercize

	void check_speed_test (float expected_speed, uint8_t status) {
		REQUIRE (get_result(6) == 0xB2);  				 // Check Mav Cmd = 178 (DO_CHANGE_SPEED)
		REQUIRE (get_result(7) == 0);
		REQUIRE (get_result(8) == status);   			 // Accepted and Executed
		REQUIRE (wp_nav.speed_in_cms == expected_speed); // Check Speed Value Is Not Changed
	}//check_speed_test

	void check_yaw_test (uint8_t status, float angle_deg, float turn_rate_dps, uint8_t relative_angle) {
		REQUIRE (get_result(6)                    == 0x73);  	     // Check Mav Cmd = 115 (CONDITION_YAW)
		REQUIRE (get_result(7)                    == 0);
		REQUIRE (get_result(8)                    == status);   	 // Accepted and Executed
		REQUIRE (control_auto_stub.angle_deg      == angle_deg);
		REQUIRE (control_auto_stub.turn_rate_dps  == turn_rate_dps);
		REQUIRE (control_auto_stub.relative_angle == relative_angle);
	}//check_speed_test

	void check_take_off_test (uint8_t status, uint8_t altitude) {
		REQUIRE (get_result(6)                    == 0x16);  	     // Check Mav Cmd = 22 (TAKE_OFF)
		REQUIRE (get_result(7)                    == 0);
		REQUIRE (get_result(8)                    == status);   	 // Accepted and Executed
		REQUIRE (take_Off_Stub.altitude == altitude);   	 // Accepted and Executed
		REQUIRE (take_Off_Stub.has_been_called);
		REQUIRE (take_Off_Stub.mode == 4);
	}//check_take_off_test
};

TEST_CASE("Speed Invalid Case", "COMMAND_LONG | DO_CHANGE_SPEED") {
	Fixture fixture;
	fixture.tab[0] = 0x00;
	fixture.tab[1] = 0x00;
	fixture.tab[2] = 0x40;
	fixture.tab[3] = 0xC0;		//-3.0 in m/s
	fixture.setup_speed_test(3.0);
	fixture.exercize();
	fixture.check_speed_test(3.0, 4);		// Command failed - Speed Value Is Not Changed
}

TEST_CASE("Speed Nominal Case", "COMMAND_LONG | DO_CHANGE_SPEED") {
	Fixture fixture;
	fixture.tab[0] = 0x00;
	fixture.tab[1] = 0x00;
	fixture.tab[2] = 0x40;
	fixture.tab[3] = 0x40;		//3.0 in m/s
	fixture.setup_speed_test(0.0);
	fixture.exercize();
	fixture.check_speed_test(300.0, 0);	// Command is Accepted and Executed - Speed Value Is Changed
}

TEST_CASE("Yaw Nominal Case (absolute)", "COMMAND_LONG | CONDITION_YAW") {
	Fixture fixture;

	control_auto_stub.angle_deg      = 0;
	control_auto_stub.turn_rate_dps  = 0;
	control_auto_stub.relative_angle = 0xFF;
	fixture.setup_yaw_test(180, 1, 0);
	fixture.exercize();
	fixture.check_yaw_test(0, 180, 1, 0);	// Command is accepted and executed - Yaw value is changed
}

TEST_CASE("Yaw Nominal Case (relative)", "COMMAND_LONG | CONDITION_YAW") {
	Fixture fixture;

	control_auto_stub.angle_deg      = 0;
	control_auto_stub.turn_rate_dps  = 0;
	control_auto_stub.relative_angle = 0xFF;
	fixture.setup_yaw_test(90, 2, 1);
	fixture.exercize();
	fixture.check_yaw_test(0, 90, 2, 1);	// Command is accepted and executed - Yaw value is changed
}

TEST_CASE("Yaw Error Case (Yaw < 0)", "COMMAND_LONG | CONDITION_YAW") {
	Fixture fixture;

	control_auto_stub.angle_deg      = 0;
	control_auto_stub.turn_rate_dps  = 0;
	control_auto_stub.relative_angle = 0xFF;
	fixture.setup_yaw_test(-0.01, 3, 0);
	fixture.exercize();
	fixture.check_yaw_test(4, 0, 0, 255);	// Command failed - Yaw value remains unchanged
}

TEST_CASE("Yaw Error Case (Yaw > 360)", "COMMAND_LONG | CONDITION_YAW") {
	Fixture fixture;

	control_auto_stub.angle_deg      = 0;
	control_auto_stub.turn_rate_dps  = 0;
	control_auto_stub.relative_angle = 0xFF;
	fixture.setup_yaw_test(360.01, 0.5, 1);
	fixture.exercize();
	fixture.check_yaw_test(4, 0, 0, 255);	// Command failed - Yaw value remains unchanged
}

TEST_CASE("Yaw Error Case (Angle mode neither absolute nor relative)", "COMMAND_LONG | CONDITION_YAW") {
	Fixture fixture;

	control_auto_stub.angle_deg      = 0;
	control_auto_stub.turn_rate_dps  = 0;
	control_auto_stub.relative_angle = 0xFF;
	fixture.setup_yaw_test(270, 1, 2);
	fixture.exercize();
	fixture.check_yaw_test(4, 0, 0, 255);	// Command failed - Yaw value remains unchanged
}

TEST_CASE("Take Off", "COMMAND_LONG | TAKE_OFF") {
	Fixture fixture;
	take_Off_Stub.has_been_called = false;
	take_Off_Stub.mode = 0;
	fixture.setup_take_off_test(8.0);
	fixture.exercize();
	fixture.check_take_off_test(0, 800.0);	// Command is accepted and executed - Take Off is started
}

