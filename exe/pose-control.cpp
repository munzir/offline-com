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
#include <fstream>
#include <vector>
#include <sstream>

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

bool input_end = true;
bool hlt_mv = false;
bool pose_mv = false;

bool lshd_reached = true;
bool rshd_reached = true;
bool llwa_reached = true;
bool rlwa_reached = true;
bool torso_reached = true;

// 0 for initial value, 1 for clockwise, 2 for counter clockwise
int lshd_dir = 0;
int rshd_dir = 0;
int llwa_dir = 0;
int rlwa_dir = 0;
int torso_dir = 0;

bool llwa_reset = false;
bool rlwa_reset = false;
bool torso_reset = false;

double llwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double rlwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_target = 0.0;

vector<double> llwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
vector<double> rlwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_default = 0.0;

int llwa_config_idx = 0;
int rlwa_config_idx = 0;

// The preset arm configurations: forward, thriller, more forward for balancing, zero
//double presetArmConfsL [][7] = {
//		{  1.102, -0.589,  0.000, -1.339,  0.000, -1.400,  0.000},
//		{  1.008,  -1.113,  -0.000,  -1.594,   0.037,   1.022,  -0.021}, // sitting grasp ready (left)
//		{  1.400, -1.000,  0.000, -0.800,  0.000, -0.800,  0.000},
//		{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
//};

//double presetArmConfsR [][7] = {
//		{ -1.102,  0.589,  0.000,  1.339,  0.000,  1.400,  0.000},
//		{  -0.855,   1.113,   0.000,   1.561,  -0.028,  -1.052,   0.021}, // sitting grasp ready (right)
//		{ -1.400,  1.000,  0.000,  0.800,  0.000,  0.800,  0.000},
//		{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
//};

vector<vector<double>> presetArmConfsL;
vector<vector<double>> presetArmConfsR;
ifstream in_file("poses.txt");
ofstream out_file("data.txt");

/* ********************************************************************************************* */
/// Read and write poses to file
void readPoseFile() {
	int count = 0;
	string line;
	while(getline(in_file,line)){
		cout << "new line" << line << endl;
		stringstream lineStream(line);
		string t;
		double d;
		vector<double> pose;
		cout << "read number ";
		while(getline(lineStream, t, ',')) {
			istringstream convert(t);
			convert >> d;
			cout << d;
			pose.push_back(d);
		}
		cout << endl;
		((count % 2) == 0) ? presetArmConfsL.push_back(pose) : presetArmConfsR.push_back(pose);
		++count;
	}
	cout << "read over" << endl;
	in_file.close();
}


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
	// Halt Arm
	cout << "halting movements" << endl;
	haltArm(daemon_cx, llwa);
	haltArm(daemon_cx, rlwa);

	// Halt Torso
	double dq [] = {0.0};
	somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, dq, 1, NULL);
	somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);

	// Halt Waist
	static Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
	somatic_metadata_set_time_now(waistDaemonCmd->meta);
	somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);
	SOMATIC_PACK_SEND( &waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
}

/* ********************************************************************************************* */
/// update waist state value
double getWaistState(){
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
/// update waist state value
double getIMUState(){
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	int r;
	double imu, imuSpeed;
	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector,
														&protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imuChan, &abstime );
	assert((imu_msg != NULL) && "Imu message is faulty!");

	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
	static const double mountAngle = -.7853981634;
	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
	double _imu = atan2(newX, imu_msg->data[2]);

	// Free the unpacked message
	somatic__vector__free_unpacked( imu_msg, &protobuf_c_system_allocator );

	return _imu;
}

/* ********************************************************************************************* */
/// record data for offline COM analysis
void recordData() {

	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_update(&daemon_cx, &rlwa);
	cout << "left arm poses: ";
	for (int i = 0; i < 7; ++i) {
		cout << llwa.pos[i] << ", ";
		out_file << llwa.pos[i] << ", ";
	}
	cout << endl;

	cout << "right arm poses: ";
	for (int i = 0; i < 7; ++i) {
		cout << rlwa.pos[i] << ", ";
		out_file << rlwa.pos[i] << ", ";
	}
	cout << endl;

	somatic_motor_update(&daemon_cx, &torso);
	cout << "torso pose: " << torso.pos[0] << endl;
	out_file << torso.pos[0] << ", ";

	double w_val = getWaistState();
	cout << "waist pose: " << w_val << endl;
	out_file << w_val << ", ";

	double imu_val = getIMUState();
	cout << "imu pose: " << imu_val << endl;
	out_file << imu_val << endl;
}

/* ********************************************************************************************* */
/// check if one of the numerical buttons is being pressed
bool buttonPressed() {
    // if we are logging
	if (b[3] == 1) {
		return true;
	}

	// arm and shoulder configs
	if (((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1)) && ((b[0] == 1) || (b[1] == 1) || (b[2] == 1))) {
		return true;
	}

	// torso commands
	if ((x[3] > 0.9) && ((b[0] == 1) || (b[1] == 1) || (b[2] == 1))) {
		return true;
	}

	// valid waist command
	if ((x[5] < -0.9) && (x[5] > 0.9)) {
		return true;
	}

	return false;
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

	double w_val = getWaistState();
	if ((x[5] < -0.9) || (x[5] > 0.9)) { cout << "waist pose: " << w_val << endl; }
}

/* ********************************************************************************************* */
/// Handles shoulders
void controlShoulders() {

	double shoulder_stepsize = 0.2;

	// button 3 for reset to deafult position
	if ((b[4] == 1) && (b[2] == 1)) {
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			pose_mv = true;

			// update target
			llwa_pos_target[0] = llwa_pos_default[0];
			cout << "new left shoulder target " << llwa_pos_target[0] << endl;

			// if we reached the previous destination, reset reached flag
			if (lshd_reached) {
				lshd_dir = 0;
				lshd_reached = false;
			}
		}
		return;
	}

	// buttons 1 and 2 are for clockwise/counterclockwise position change
	if (((b[4]==1) && (b[0] == 1)) || ((b[4]==1) && (b[1] == 1))) {
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			pose_mv = true;

			// we only update our target if we have reach our previous target or the direction is difference
			if (lshd_reached || (((b[0] == 1) && (lshd_dir == -1)) || ((b[1] == 1) && (lshd_dir == 1)))) {
			    if (b[0] == 1) {
			    	lshd_dir = 1;
			    } else {
			    	lshd_dir = -1;
			    }
				llwa_pos_target[0] += (shoulder_stepsize * lshd_dir);
			    cout << "new left shoulder target " << llwa_pos_target[0] << endl;
				lshd_reached = false;
			}
		}
		return;
	}

	// button 3 for reset to deafult position
	if ((b[5] == 1) && (b[2] == 1)) {
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			pose_mv = true;

			// update target
			rlwa_pos_target[0] = rlwa_pos_default[0];
			cout << "new right shoulder target " << rlwa_pos_target[0] << endl;

			// if we reached the previous destination, reset reached flag
			if (rshd_reached) {
			    rshd_dir = 0;
				rshd_reached = false;
			}
		}
		return;
	}

	// buttons 1 and 2 are for clockwise/counterclockwise position change
	if (((b[5]==1) && (b[0] == 1)) || ((b[5]==1) && (b[1] == 1))) {
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			pose_mv = true;

			// we only update our target if we have reach our previous target or the direction is difference
			if (rshd_reached || (((b[0] == 1) && (rshd_dir == -1)) || ((b[1] == 1) && (rshd_dir == 1)))) {
				if (b[0] == 1) {
					rshd_dir = 1;
				} else {
					rshd_dir = -1;
				}
				rlwa_pos_target[0] += (shoulder_stepsize * rshd_dir);
				cout << "new right shoulder target " << rlwa_pos_target[0] << endl;
				rshd_reached = false;
			}
		}
		return;
	}
}

void updateArmTarget(double targetPose[], vector<double> configPose) {
	// we are ignoring shoulder motor (idx = 0)
	for (int i = 1; i < 7; ++i) {
		targetPose[i] = configPose[i];
	}
}
void printArmPos(double pos[], string dir) {
	cout << "new " << dir << " arm target";
	for (int i =0; i < 7; ++i) {
		cout << pos[i];
	}
	cout << endl;
}
/* ********************************************************************************************* */
/// Handles arm configurations
void controlArms() {

	// button 3 for reset to deafult position
	if ((b[6] == 1) && (b[2] == 1)) {
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			pose_mv = true;
			// update target
			updateArmTarget(llwa_pos_target, llwa_pos_default);
			printArmPos(llwa_pos_target, "left");
			// if we reached the previous destination, reset reached flag
			if (llwa_reached) {
				llwa_reached = false;
				llwa_dir = 0;
			}
		}
		return;
	}

	// buttons 1 and 2 are for clockwise/counterclockwise position change
	if (((b[6]==1) && (b[0] == 1)) || ((b[6]==1) && (b[1] == 1))) {
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			pose_mv = true;
			// we only update our target if we have reach our previous target or the direction is difference
			if (llwa_reached || (((b[0] == 1) && (llwa_dir == -1)) || ((b[1] == 1) && (llwa_dir == 1)))) {
				// if reached previous location, decrease/increase target by step
                if (b[0] == 1) {
                	llwa_dir = 1;
                } else {
                	llwa_dir = -1;
                }
				llwa_config_idx = (llwa_config_idx + llwa_dir) % sizeof(presetArmConfsL);
				updateArmTarget(llwa_pos_target, presetArmConfsL[llwa_config_idx]);
				printArmPos(llwa_pos_target, "left");
				lshd_reached = false;
			}
		}
		return;
	}

	// button 3 for reset to deafult position
	if ((b[7] == 1) && (b[2] == 1)) {
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			pose_mv = true;
			// update target
			updateArmTarget(rlwa_pos_target, rlwa_pos_default);
			printArmPos(rlwa_pos_target, "right");
			// if we reached the previous destination, reset reached flag
			if (rlwa_reached) {
				rlwa_reached = false;
				rlwa_dir = 0;
			}
		}
		return;
	}

	// buttons 1 and 2 are for clockwise/counterclockwise position change
	if (((b[7]==1) && (b[0] == 1)) || ((b[6]==1) && (b[1] == 1))) {
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			pose_mv = true;

			if (rlwa_reached || (((b[0] == 1) && (rlwa_dir == -1)) || ((b[1] == 1) && (rlwa_dir == 1)))) {
				// if reached previous location, decrease/increase target by step
				if (b[0] == 1) {
					rlwa_dir = 1;
				} else {
					rlwa_dir = -1;
				}
				rlwa_config_idx = (rlwa_config_idx + rlwa_dir) % sizeof(presetArmConfsR);
				updateArmTarget(rlwa_pos_target, presetArmConfsR[rlwa_config_idx]);
				printArmPos(rlwa_pos_target, "right");
				rlwa_reached = false;
			}
		}
		return;
	}
}

/* ********************************************************************************************* */
/// Handles torso
void controlTorso() {

	double torso_stepsize = 0.1;

	// button 3 for reset to deafult position
	if ((x[3] > 0.9) && (b[2] == 1)) {
		if(!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			torso_reset = true;
			pose_mv = true;

			// update target
			torso_pos_target = torso_pos_default;
			cout << "new torso target " << torso_pos_target << endl;

			// if we reached the previous destination, reset reached flag
			if (torso_reached) {
				torso_reached = false;
				torso_dir = 0;
			}
		}
		return;
	}

	// buttons 1 and 2 are for forward/backward position toggles
	if (((x[3] > 0.9) && (b[0] == 1)) || ((x[3] > 0.9) && (b[1] == 1))) {
		if (!pose_mv && input_end) {
			// if not moving currently, update reset and move flags
			torso_reset = true;
			pose_mv = true;

			if (torso_reached || (((x[3] > 0.9) && (torso_dir == -1)) || ((x[3] > 0.9) && (torso_dir == 1)))) {
				// if reached previous location, decrease/increase target by step
                if (b[0] == 1) {
                	torso_dir = 1;
                } else {
                	torso_dir = -1;
                }
				torso_pos_target += (torso_stepsize * torso_dir);
                cout << "new torso target " << torso_pos_target << endl;
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
	if (b[3] == 1) recordData();

	// if no buttons are actively pressed we halt all movements
	if (buttonPressed()) {
		input_end = false;
	} else {
		if (pose_mv) { // released button while moving
			hlt_mv = true;
			pose_mv = false;
		}
		input_end = true;
	}

}

/* ********************************************************************************************* */
/// check each body part and send motor reset/movement commands (other than waist)
void applyMove() {

	if (hlt_mv) {
		haltMovement();
		hlt_mv = false;
	}

	if (llwa_reset) {
	    cout << "resetting left arm" << endl;
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
		double torso_pos_array[1] = {torso_pos_target};
		somatic_motor_cmd(&daemon_cx, &llwa, POSITION, llwa_pos_target, 7, NULL);
		somatic_motor_cmd(&daemon_cx, &rlwa, POSITION, rlwa_pos_target, 7, NULL);
		somatic_motor_cmd(&daemon_cx, &torso, POSITION, torso_pos_array, 1, NULL);
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

	double delta=0.01;


	// check shoulders
	if ((fabs(llwa.pos[0] - llwa_pos_target[0]) < delta) && !lshd_reached) {
		cout << "left shoulder target reached" << endl;
		lshd_reached = true;
	}


	if ((fabs(rlwa.pos[0] - rlwa_pos_target[0]) < delta) && !rshd_reached) {
		cout << "right shoulder target reached" << endl;
		rshd_reached = true;
	}

	// check arm configuration
    if(checkArm(llwa.pos, llwa_pos_target, delta) && !llwa_reached) {
		cout << "left arm target reached" << endl;
		llwa_reached = true;

	}

	if(checkArm(rlwa.pos, rlwa_pos_target, delta) && !rlwa_reached) {
		cout << "right arm target reached" << endl;
		rlwa_reached = true;
	}

	// check torso configuration
	if ((fabs(torso.pos[0] - torso_pos_target) < delta) && !torso_reached) {
		cout << "torso target reached" << endl;
		double dq[] = {0.0};
		somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, dq, 1, NULL);
		somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
		torso_reached = true;
	}
}

/* ********************************************************************************************* */
/// handles somatic signals
void run() {
	//get initial states
	init_pos_targ();

	readPoseFile();

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

	// close our write file
	out_file.close();
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

