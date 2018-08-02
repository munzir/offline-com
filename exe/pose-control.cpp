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

#define TORSO_STEP 0.1
#define SHOULDER_STEP 0.1
#define POSE_ERROR 0.01

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
bool llwa_mv = false;
bool rlwa_mv = false;
bool torso_mv = false;

bool llwa_reached = true;
bool rlwa_reached = true;
bool torso_reached = true;

// 0 for initial value, 1 for clockwise, 2 for counter clockwise
int llwa_dir = 0;
int rlwa_dir = 0;
int torso_dir = 0;

bool llwa_reset = false;
bool rlwa_reset = false;
bool torso_reset = false;

double llwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double rlwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_target = 0.0;

//ifstream myfile ("/home/munzir/areeb/waist_log.txt", ios::in);
string inputline;
//double get_current();

vector<double> llwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
vector<double> rlwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_default = 0.0;

int llwa_config_idx = 0;
int rlwa_config_idx = 0;
int torso_config_idx = 0;

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
vector<double> presetTorsoConfs;

ifstream in_file("../data/poseTrajectoriesrfinalSet/interposeTraj1-2.txt");
ofstream out_file("../data/interposeTraj1-2out.txt", ios::app);

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/* ********************************************************************************************* */
/// Read and write poses to file
void readPoseFile() {
	int count = 0;
	string line;
	presetArmConfsL.push_back(llwa_pos_default);
	presetArmConfsR.push_back(rlwa_pos_default);
	presetTorsoConfs.push_back(torso_pos_default);
	while(getline(in_file,line)) {
		cout << "new line" << line << endl;
		stringstream lineStream(line);
		string t;
		double d;
		vector<double> poseL;
		vector<double> poseR;
		int i = 0;
		cout << "read number ";
		while (getline(lineStream, t, ' ')) {
			if(!t.empty()) {
				istringstream convert(t);
				convert >> d;
				cout << d;
				if (i == 9) {
					presetTorsoConfs.push_back(d);
				}
				if (i > 10 && i <= 17) {
					poseL.push_back(d);
				}
				if (i > 17) {
					poseR.push_back(d);
				}
				i++;
			}
		}
		cout << endl;
		presetArmConfsL.push_back(poseL);
		presetArmConfsR.push_back(poseR);
		++count;
	}
	cout << "read over" << endl;
	cout << "total configurations read:" << count << endl;
	in_file.close();

	cout << "left arm configurations: " << endl;
	for (auto c : presetArmConfsL) {
		for (auto d : c) {
			cout << d << ", ";
		}
		cout << endl;
	}

	cout << "right arm configurations: " << endl;
	for (auto c : presetArmConfsR) {
		for (auto d : c) {
			cout << d << ", ";
		}
		cout << endl;
	}

	cout << "torso configurations" << endl;
	for (auto c : presetTorsoConfs) {
		cout << c << endl;
	}
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
/// update waist current value
double getWaistCurr(){
	struct timespec currTime;
	clock_gettime(CLOCK_MONOTONIC, &currTime);	
	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
	int r;
	Somatic__MotorState *waist = NULL;
	while(waist == NULL) waist = getMotorMessage(waistChan);
	double waist_curr = (waist->current->data[0]);
	return waist_curr;
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
	
	double w_curr = getWaistCurr();
	cout << "waist current: " << w_curr << endl;
	out_file << w_curr << ", ";
	//string line;
	ifstream myfile ("/home/munzir/areeb/waist_log.txt", ios::in);
	if(myfile.is_open()){
		getline(myfile, inputline);
//		getline(myfile, inputline);
		cout << inputline << endl;
	}
	else{
		cout << "WAIST LOG FILE NOT OPEN" << endl;
	}
	//out_file << "waist current: " << get_current() << endl;
}

/* ********************************************************************************************* */
/// if D key is pressed, record krang state data
void *kbhit(void *) {
	char input;
	double kOffset = 0.05;
	while (true) {
		input = cin.get();
		pthread_mutex_lock(&mutex);
		if (input == 'd') {
			recordData();
		};
		if (input == 'x') {
			out_file << "DELETE PREVIOUS LINE" << endl;
		};
		pthread_mutex_unlock(&mutex);
	}
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

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ********************************************************************************************* */
/// Set Torso Target
void resetTorsoTarget() {
	somatic_motor_update(&daemon_cx, &torso);
	torso_pos_target = torso.pos[0];
}

/* ********************************************************************************************* */
/// Set left arm Target
void resetLlwaTarget() {
	for (int i = 0; i < 7; ++i) {
		llwa_pos_target[i] = llwa.pos[i];
	}
}

/// Set right arm Target
void resetRlwaTarget() {
	for (int i = 0; i < 7; ++i) {
		rlwa_pos_target[i] = rlwa.pos[i];
	}
}

/* ********************************************************************************************* */
/// Set initial position targets for arms and torso to existing positions
void init_pos_targ() {
	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_update(&daemon_cx, &rlwa);
	resetLlwaTarget();
	resetRlwaTarget();
	resetTorsoTarget();
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

//	// Halt Waist
//	static Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
//	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
//	somatic_metadata_set_time_now(waistDaemonCmd->meta);
//	somatic_metadata_set_until_duration(waistDaemonCmd->meta, .1);
//	SOMATIC_PACK_SEND( &waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
}

/* ********************************************************************************************* */
/// check if one of the numerical buttons is being pressed
bool buttonPressed() {
    // if we are logging
	if (b[9] == 1) {
		return true;
	}

	bool command_btn = ((b[0] == 1) || (b[1] == 1) || (b[2] == 1) || (b[3] == 1));

	bool part_btn = ((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1) || (x[1] > 0.9));

	// arm, shoulder & torso configs
	if (part_btn && command_btn)  {
		return true;
	}

	// valid waist command
	if (((x[5] < -0.9) || (x[5] > 0.9)) && command_btn) {
		return true;
	}

	return false;
}


void updateArmTarget(double targetPose[], vector<double> configPose) {
	for (int i = 0; i < 7; ++i) {
		targetPose[i] = configPose[i];
	}
}

void printArmPos(double pos[], string dir) {
	cout << "new " << dir << " arm target";
	for (int i =0; i < 7; ++i) {
		cout << pos[i] << ", ";
	}
	cout << endl;
}

/* ********************************************************************************************* */
/// Handles arm configurations
void controlArms() {

	// button 4 for reset current target
	if ((b[6] == 1) && (b[3] == 1)) {
		resetLlwaTarget();
	}

	// button 2 for reset to deafult position
	if ((b[6] == 1) && (b[1] == 1)) {
		if(!llwa_mv && input_end) {
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			llwa_mv = true;
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

	// buttons 1 and 3 are for clockwise/counterclockwise position change
	if (((b[6]==1) && (b[0] == 1)) || ((b[6]==1) && (b[2] == 1))) {
		if (!llwa_mv && input_end) {
		    cout << "new left arm config" << endl;
			// if not moving currently, update reset and move flags
			llwa_reset = true;
			llwa_mv = true;
			// we only update our target if we have reach our previous target or the direction is difference
			if (llwa_reached || (((b[0] == 1) && (llwa_dir == -1)) || ((b[2] == 1) && (llwa_dir == 1)))) {
				// if reached previous location, decrease/increase target by step
                if (b[0] == 1) {
                	llwa_dir = 1;
                } else {
                	llwa_dir = -1;
                }
                cout << "left arm config id " << llwa_config_idx;
				cout << "size of preset config " <<(presetArmConfsL.size());
				llwa_config_idx = (llwa_config_idx + llwa_dir) % presetArmConfsL.size();
				cout << "left arm config id " << llwa_config_idx;
				updateArmTarget(llwa_pos_target, presetArmConfsL[llwa_config_idx]);
				printArmPos(llwa_pos_target, "left");
				llwa_reached = false;
			}
		}
		return;
	}

	// button 4 for reset current target
	if ((b[7] == 1) && (b[3] == 1)) {
		resetRlwaTarget();
	}

	// button 2 for reset to deafult position
	if ((b[7] == 1) && (b[1] == 1)) {
		if(!rlwa_mv && input_end) {
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			rlwa_mv = true;
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

	// buttons 1 and 3 are for clockwise/counterclockwise position change
	if (((b[7]==1) && (b[0] == 1)) || ((b[7]==1) && (b[2] == 1))) {
		if (!rlwa_mv && input_end) {
			cout << "new right arm config" << endl;
			// if not moving currently, update reset and move flags
			rlwa_reset = true;
			rlwa_mv = true;

			if (rlwa_reached || (((b[0] == 1) && (rlwa_dir == -1)) || ((b[2] == 1) && (rlwa_dir == 1)))) {
				// if reached previous location, decrease/increase target by step
				if (b[0] == 1) {
					rlwa_dir = 1;
				} else {
					rlwa_dir = -1;
				}
				rlwa_config_idx = (rlwa_config_idx + rlwa_dir) % presetArmConfsR.size();
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

	// button 4 for reset current target
	if ((x[1] > 0.9) && (b[3] == 1)) {
		resetTorsoTarget();
	}

	// button 2 for reset to deafult position
	if ((x[1] > 0.9) && (b[1] == 1)) {
		if(!torso_mv && input_end) {
			// if not moving currently, update reset and move flags
			torso_reset = true;
			torso_mv = true;

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

	// buttons 1 and 3 are for forward/backward position toggles
	if (((x[1] > 0.9) && (b[0] == 1)) || ((x[1] > 0.9) && (b[2] == 1))) {
		if (!torso_mv && input_end) {
			// if not moving currently, update reset and move flags
			torso_reset = true;
			torso_mv = true;

			if (torso_reached || (((b[0] == 1) && (torso_dir == -1)) || ((b[2] == 1) && (torso_dir == 1)))) {
				// if reached previous location, decrease/increase target by step
                if (b[0] == 1) {
                	torso_dir = 1;
                } else {
                	torso_dir = -1;
                }
                torso_config_idx = (torso_config_idx + torso_dir) % presetTorsoConfs.size();
				torso_pos_target = presetTorsoConfs[torso_config_idx];
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

    controlArms();

    controlTorso();

	// if no buttons are actively pressed we halt all movements
	if (buttonPressed()) {
		input_end = false;
	} else {
		pose_mv = (llwa_mv || rlwa_mv || torso_mv);
		if (pose_mv) { // released button while moving
			hlt_mv = true;
			pose_mv = false;
			llwa_mv = false;
			rlwa_mv = false;
			torso_mv = false;
		}
		input_end = true;
	}

}

/* ********************************************************************************************* */
/// move arm
void moveArm(somatic_motor_t &arm, double target[] ) {
	double arm_target[7] = {0, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 7; ++i) {
		arm_target[i] = target[i];
	}
	somatic_motor_cmd(&daemon_cx, &arm, POSITION, arm_target, 7, NULL);
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
		cout << "resetting right arm" << endl;
		somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
		rlwa_reset = false;
	}

	if (torso_reset) {
		somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
		torso_reset = false;
	}

	usleep(1e5);

	if (llwa_mv) {
		somatic_motor_update(&daemon_cx, &llwa);
		moveArm(llwa, llwa_pos_target);
	}
	if (rlwa_mv) {
		somatic_motor_update(&daemon_cx, &rlwa);
		moveArm(rlwa, rlwa_pos_target);
	}
	if (torso_mv) {
		double torso_pos_array[1] = {torso_pos_target};
		somatic_motor_cmd(&daemon_cx, &torso, POSITION, torso_pos_array, 1, NULL);
	}
}

/* ********************************************************************************************* */
/// check the 7 motors (indexed 0 to 6) of the arm position and match with pose target
bool checkArm(double pos[7], double target[7], double del) {
   for (int i = 0; i < 7; ++i) {
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

	// check arm configuration
    if(checkArm(llwa.pos, llwa_pos_target, POSE_ERROR) && !llwa_reached) {
		cout << "left arm target reached" << endl;
		llwa_reached = true;

	}

	if(checkArm(rlwa.pos, rlwa_pos_target, POSE_ERROR) && !rlwa_reached) {
		cout << "right arm target reached" << endl;
		rlwa_reached = true;
	}

	// check torso configuration
	if ((fabs(torso.pos[0] - torso_pos_target) < POSE_ERROR) && !torso_reached) {
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

