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
#define getMotorMessage(x) (SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, \
	&protobuf_c_system_allocator, 1024, &x, &abstime))

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

bool hlt = false;
bool input_end = false;

bool pose_mv = false;

bool lshd_reached = true;
bool rshd_reached = true;
bool llwa_reached = true;
bool rlwa_reached = true;
bool torso_reached = true;

bool llwa_reset = false;
bool rlwa_reset = false;
bool torso_reset = false;

double llwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double rlwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_target = 0.0;

double llwa_pos_default[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double rlwa_pos_default[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_default = -5.26;

// The preset arm configurations: forward, thriller, more forward for balancing, zero
double presetArmConfsL [][7] = {
		{  1.102, -0.589,  0.000, -1.339,  0.000, -1.400,  0.000},
		{  1.008,  -1.113,  -0.000,  -1.594,   0.037,   1.022,  -0.021}, // sitting grasp ready (left)
		{  1.400, -1.000,  0.000, -0.800,  0.000, -0.800,  0.000},
		{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
};
double presetArmConfsR [][7] = {
		{ -1.102,  0.589,  0.000,  1.339,  0.000,  1.400,  0.000},
		{  -0.855,   1.113,   0.000,   1.561,  -0.028,  -1.052,   0.021}, // sitting grasp ready (right)
		{ -1.400,  1.000,  0.000,  0.800,  0.000,  0.800,  0.000},
		{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
};

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

/* ********************************************************************************************* */
/// Initialize arms and torso with somatic motors. Initialize waist command/state, imu, and joystick channels
void init() {

	// initialize daemon with options
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "pose-ctrl";
	somatic_d_init( &daemon_cx, &dopt);

	// Initialize the arms and torso module
	initArm(daemon_cx, llwa, "llwa");
	initArm(daemon_cx, rlwa, "rlwa");
	initTorso(daemon_cx, torso);

	// Initialize the waist channels
	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &waistCmdChan, "waistd-cmd", NULL);

	// Initialize IMU Data
	initIMU(daemon_cx, imuChan);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n",
				   ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

}

/* ********************************************************************************************* */
/// Set initial position targets for arms and torso to existing positions
void init_pos_targ() {

	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_update(&daemon_cx, &rlwa);
	for (int i = 0; i < 7; ++i) {
		llwa_pos_target[i] = llwa.pos[i];
	}
	for (int i = 0; i < 7; ++i) {
		rlwa_pos_target[i] = rlwa.pos[i];
	}

	somatic_motor_update(&daemon_cx, &torso);
	torso_pos_target = torso.pos[0];

}

/* ********************************************************************************************* */
/// stop all movements
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

/* ********************************************************************************************* */
/// update waist state value
double updateWaistState(){
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	int r;
	Somatic__MotorState *waist = NULL;
	while(waist == NULL) waist = getMotorMessage(waistChan);
	double waist_val = (waist->position->data[0] - waist->position->data[1]) / 2.0;
	return waist_val;
}

/* ********************************************************************************************* */
/// record data for offline COM analysis
void recordData() {

	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_update(&daemon_cx, &rlwa);
	cout << "left arm poses: ";
	for (int i = 0; i < 7; ++i) {
		cout << llwa.pos[i] << ", ";
	}
	cout << endl;

	cout << "right arm poses: ";
	for (int i = 0; i < 7; ++i) {
		cout << rlwa.pos[i] << ", ";
	}
	cout << endl;

	somatic_motor_update(&daemon_cx, &torso);
	cout << "torso pose: " << torso.pos[0] << endl;

	double w_val = updateWaistState();
	cout << "waist pose: " << w_val << endl;
}

/* ********************************************************************************************* */
/// check if one of the numerical buttons is being pressed
bool buttonPressed() {
	bool pressed = false;
	for (int i =0; i < 10; ++i) {
		if (b[i] == 1) {
			pressed = true;
			return pressed;
		}
	}
	return pressed;
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the waist module
void controlWaist() {

	// Set the mode we want to send to the waist daemon, waist only moves when we are push joystick to max
	Somatic__WaistMode waistMode;
	if(x[5] < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(x[5] > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
	else waistMode = SOMATIC__WAIST_MODE__STOP;

	// Send message to the krang-waist daemon
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	int r = SOMATIC_PACK_SEND(&waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n",
							ach_result_to_string(static_cast<ach_status_t>(r)));

	double w_val = updateWaistState();
	cout << "waist pose: " << w_val << endl;
}

/* ********************************************************************************************* */
/// Handles shoulders
void controlShoulders() {

	double shoulder_stepsize;

	// button 3 for reset to deafult position
	if ((b[4] == 1) && (b[2] == 1)) {
		input_end = false;
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			pose_mv = true;

			// update target
			llwa_pos_target[0] = llwa_pos_default[0];

			// if we reached the previous destination, reset reached flag
			if (lshd_reached) {
				lsdh_reached = false;
			}
		}
		return;
	}

	// buttons 1 and 2 are for forward/backward position toggles
	if (((b[4]==1) && (b[0] == 1)) || ((b[4]==1) && (b[1] == 1))) {
		input_end = false;
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			pose_mv = true;

			if (lshd_reached) {
				// if reached previous location, decrease/increase target by step
				double step = (b[0] == 1) ? shoulder_stepsize: -shoulder_stepsize;
				llwa_pos_target[0] += step_size;
				lsdh_reached = false;
			}
		}
		return;
	}

	// button 3 for reset to deafult position
	if ((b[5] == 1) && (b[2] == 1)) {
		input_end = false;
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			pose_mv = true;

			// update target
			rlwa_pos_target[0] = rlwa_pos_default[0];

			// if we reached the previous destination, reset reached flag
			if (rshd_reached) {
				rsdh_reached = false;
			}
		}
		return;
	}

	// buttons 1 and 2 are for forward/backward position toggles
	if (((b[5]==1) && (b[0] == 1)) || ((b[5]==1) && (b[1] == 1))) {
		input_end = false;
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			pose_mv = true;

			if (rshd_reached) {
				// if reached previous location, decrease/increase target by step
				double step = (b[0] == 1) ? shoulder_stepsize : -shoulder_stepsize;
				rlwa_pos_target[0] += step_size;
				rsdh_reached = false;
			}
		}
		return;
	}
}

/* ********************************************************************************************* */
/// Handles arm configurations
void controlArms() {

}

/* ********************************************************************************************* */
/// Handles torso
void controlTorso() {

	double torso_stepsize = 0.1;

	// button 3 for reset to deafult position
	if ((x[3] > 0.9) && (b[2] == 1)) {
		input_end = false;
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			torso_reset = true;
			pose_mv = true;

			// update target
			torso_pos_target = torso_pos_default;

			// if we reached the previous destination, reset reached flag
			if (torso_reached) {
				torso_reached = false;
			}
		}
		return;
	}

	// buttons 1 and 2 are for forward/backward position toggles
	if (((x[3] > 0.9) && (b[0] == 1)) || ((x[3] > 0.9) && (b[1] == 1))) {
		input_end = false;
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			torso_reset = true;
			pose_mv = true;

			if (torso_reached) {
				// if reached previous location, decrease/increase target by step
				double step = (b[0] == 1) ? torso_stepsize : -torso_stepsize;
				torso_pos_target[0] += step_size;
				torso_reached = false;
			}
		}
		return;
	}
}


/* ********************************************************************************************* */
/// handles joystick commands
void processJS() {

    controlWaist();

    controlShoulders();

    controlArms();

    controlTorso();

	// When button 4 is pressed we record our pose
	if (b[3] == 1) {
		recordData();
	}

	// if no buttons are actively pressed we halt all movements
	if (!buttonPressed()) {
		input_end = false;
		pose_mv = false;
		hlt = true;
		return;
	}

	input_end = true;
}

/* ********************************************************************************************* */
/// check each body part and send motor reset/movement commands (other than waist)
void applyMove() {
	if (llwa_reset) {
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
		llwa_reset = false;
	}

	if (rlwa_reset) {
		somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
		rlwa_reset = false;
	}

	if (torso_reset) {
		somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
		torso_reset = false;
	}

	if (pose_mv) {
		somatic_motor_cmd(&daemon_cx, &llwa, POSITION, llwa_pos_target, 7, NULL);
		somatic_motor_cmd(&daemon_cx, &rlwa, POSITION, rlwa_pos_target, 7, NULL);
		somatic_motor_cmd(&daemon_cx, &torso, POSITION, torso_pos_target, 1, NULL);
	}
}

/* ********************************************************************************************* */
/// check the 6 motors (indexed 1 to 6) of the arm position and match with pose target
bool checkArm(double pos[7], double target[7], double del) {
   for (int i = 1; i < 7; ++i) {
       if (fabs(pos[i] - target[i]) >= del) {
           return false;
       }
   }
   return true;
}

/* ********************************************************************************************* */
/// Update poses and stop movements if we have reached our goals
void poseUpdate() {
	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_update(&daemon_cx, &rlwa);
	somatic_motor_update(&daemon_cx, &torso);

	double delta=0.05;

	// check shoulders
	if (fabs(llwa.pos[0] - llwa_pos_target[0]) < delta) {
		lshd_reached = true;
	}

	if (fabs(rlwa.pos[0] - rlwa_pos_target[0]) < delta) {
		rshd_reached = true;
	}

	// check arm configuration
    llwa_reached = (llwa_pos, llwa_pos_target, delta);
	rlwa_reached = (rlwa_pos, rlwa_pos_target, delta);

	// half if arms positions reached
	if (lshd_reached && llwa_reached) {
		somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	}

	if (rshd_reached && rlwa_reached) {
		somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	}

	// check torso configuration
	if (fabs(torso.pos[0] - torso_pos_target) < delta) {
		somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
		torso_reached = true;
	}
}

/* ********************************************************************************************* */
/// handles somatic signals
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

/* ********************************************************************************************* */
/// close channels and daemons
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

