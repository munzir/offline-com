/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file shoulder-demo.cpp
 * @author Shimin Zhang
 * @date July 13, 2013
 * @brief This file demonstrates the use of the Logitech joystick controller to control the Schunk
 * modules and Robotiq grippers.
 */

#include "initModules.h"
#include "motion.h"
#include <iostream>

using namespace std;

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION

// Init somatic daemon
somatic_d_t daemon_cx;

// Init ACH channels  
ach_channel_t js_chan;			/// the state channel for the joystick module
ach_channel_t waistChan;		/// the state channel for the waist module
ach_channel_t waistCmdChan;		/// the state channel for the waist module
ach_channel_t imuChan; 			/// the state channel for IMU data

// Init somatic motor objects
somatic_motor_t llwa, rlwa, torso;

// Why do we need to allocate memory for waist command but not others?
Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();		///< The waist command

// Init controller signals, b for buttons and x for axes
char b[10];
double x[6];

bool lshd_mv = false;
bool lshd_reached = false;
bool lshd_hlt = false;
bool lshd_reset = false;
bool input_end = false;
double llwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double lshd_default = 0.00;

/* ********************************************************************************************* */
/// Reads the joystick data into global variables 'b' and 'x', b for button press and x for axes data
void readJoystick() {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg =
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

	// Save values from joystick message buttons and save them to b
	for(size_t i = 0; i < 10; i++)
		b[i] = js_msg->buttons->data[i] ? 1 : 0;

	// Copy over axes data
	memcpy(x, js_msg->axes->data, sizeof(x));

	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
}

void init() {

	// initialize daemon with options
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "lshd-ctrl";
	somatic_d_init( &daemon_cx, &dopt);

	// Initialize the arms and torso module
	initArm(daemon_cx, llwa, "llwa");
	initArm(daemon_cx, rlwa, "rlwa");
	initTorso(daemon_cx, torso);

	// Initialize the waist channels
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &waistCmdChan, "waistd-cmd", NULL);

	// Initialize IMU Data
	initIMU(daemon_cx, &imuChan);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

}

void init_pos_targ() {
	somatic_motor_update(&daemon_cx, &llaw);
	for (int i = 0; i < 7; ++i) {
		llwa_pos_target[i] = llaw.pos[i];
	}
}

void haltMovement () {
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
	static Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
	somatic_metadata_set_time_now(waistDaemonCmd->meta);
	somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);
	SOMATIC_PACK_SEND( &waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
}

void processJS() {

    // button 9 halts all motors
	if ((b[8] == 1)) {
		input_end = false;
		haltMovement();
	}

	// button 3 for reset to deafult position
	if ((b[4] == 1) && (b[2] == 1)) {
		input_end = false;
		if(!lshd_mv && input_end) {
		    // if not moving currently, update reset and move flags
			lshd_reset = true;
			lshd_mv = true;

			// update target
			llwa_pos_target[0] = lshd_default;

			// if we reached the previous destination, reset reached flag
			if (lshd_reached) {
				lshd_reached = false;
			}
		}
		return;
	}

	// buttons 1 and 2 are for forward/backward position toggles
	if (((b[4]==1) && (b[0] == 1)) || ((b[4]==1) && (b[1] == 1))) {
		input_end = false;
		if (!lshd_mv && input_end) {
			// if not moving currently, update reset and move flags
			lshd_reset = true;
			lshd_mv = true;

			if (lshd_reached) {
				// if reached previous location, decrease/increase target by step
				double step_size = (b[0] == 1) ? 0.1 : -0.1;
				llwa_pos_target[0] += step_size;
				lshd_reached = false;
			}
		}
		return;
	}

	if ((b[4] == 0)) {
		input_end = false;
		// trigger halt command and stop moving
		lshd_mv = false;
		lshd_hlt = true;
		return;
	}

	input_end = true;
}

void applyMove() {
	if (lshd_reset) {
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
		lshd_reset = false;
	}
	if (lshd_mv) {
		somatic_motor_cmd(&daemon_cx, &llwa, POSITION, llwa_pos_target, 7, NULL);
	}
}

void poseUpdate() {
	somatic_motor_update(&daemon_cx, &llwa);

	if (fabs(llwa.pos[0] - llwa_pos_target[0]) < 0.1) {
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
		lshd_reached = true;
		lshd_mv = false;
	}
}

void run() {
	//get initial states
	init_pos_targ();

	// Unless an interrupt or terminate message is received, process the new message
	while(!somatic_sig_received) {

		// Read the joystick data
		readJoystick();

		processJS();

		applyMove();

		poseUpdate();

		usleep(1e4);
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

void destroy() {
	// Halt the waist modules
	static Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);

	// Do we need meta data?
	somatic_metadata_set_time_now(waistDaemonCmd->meta);
	somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);

	ach_status_t r = SOMATIC_PACK_SEND( &waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", ach_result_to_string(r));
	somatic_d_channel_close(&daemon_cx, &waistChan);
	somatic_d_channel_close(&daemon_cx, &waistCmdChan);

	// Halt the Schunk modules
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);

	// close imu state channels
	somatic_d_channel_close(&daemon_cx, &imuChan);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	cout << "Destroyed and halted" << endl;
}

int main() {
	
	init();
	run();
	destroy(); 
	
	return 0; 
}

