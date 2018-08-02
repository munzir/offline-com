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
 * @file pose-full-control.cpp
 * @author Shimin Zhang
 * @modifier Akash Patel
 * @date August 1, 2018
 * @brief This file moves Krang to a specified pose from an input file using the Logitech joystick controller
 */

// // Includes
#include "initModules.h"
#include "motion.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

// // Namespaces
using namespace std;

// // Defines
#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define getMotorMessage(x) (SOMATIC_WAIT_LAST_UNPACK(r, somatic__motor_state, \
    &protobuf_c_system_allocator, 1024, &x, &abstime))

// tolerance for the pose
// TODO Test lower tolerance value
#define POSE_TOL= 0.01
//#define POSE_TOL= 0.005

// // Global Stuff
// Init somatic daemon
somatic_d_t daemon_cx;

// Init ACH channels
ach_channel_t js_chan;      // the state channel for the joystick module
ach_channel_t waistChan;    // the state channel for the waist module
ach_channel_t waistCmdChan; // the state channel for the waist module //TODO why same description? Should it be 'command channel' instead
ach_channel_t imuChan;      // the state channel for IMU data

// Init somatic motor objects
somatic_motor_t llwa, rlwa, torso;

// Why do we need to allocate memory for waist command but not others?
Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();  // The waist command

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

// TODO I believe below comment is incorrect
// 0 for initial value, 1 for clockwise, 2 for counter clockwise
// -1 for backwards direction and 1 for positive direction in pose file
int llwa_dir = 0;
int rlwa_dir = 0;
int torso_dir = 0;

bool llwa_reset = false;
bool rlwa_reset = false;
bool torso_reset = false;

double llwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double llwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_target = 0.0;

vector<double> llwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
vector<double> rlwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_default = 0.0;

// TODO what does this mean
// Index of the current pose in the read file
int llwa_config_idx = 0;
int rlwa_config_idx = 0;
int torso_config_idx = 0;

vector<vector<double>> presetArmConfsL;
vector<vector<double>> presetArmConfsR;
vector<double> presetTorsoConfs;

ifstream poses_in_file("../data/poses.txt");
ofstream pose_out_file("../data/out.txt", ios::app);

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************/
// Read and write poses to file
void readPoseFile() {
    int count = 0;
    string line;
    presetArmConfsL.push_back(llwa_pose_default);
    presetArmConfsR.push_back(rlwa_pos_default);
    presetTorsoConfs.push_back(torso_pos_default);
    while (getline(poses_in_file, line)) {
        cout << "new line: " << line << endl;
        stringstream lineStream(line);
        string strNum;
        double doubNum;
        vector<double> poseL;
        vector<double> poseR;
        int i = 0;
        cout << "read number: ";
        while (getline(lineStream, strNum, ' ')) {
            if (!strNum.empty()) {
                istringstream convert(strNum);
                convert >> doubNum;
                cout << doubNum;
                // TODO change indexes to reflect munzir format
                if (i == 9) {
                    presetTorsoConfs.push_back(doubNum);
                }
                if (i > 10 && i <= 17) {
                    poseL.push_back(doubNum);
                }
                if (i > 17) {
                    poseR.push_back(doubNum);
                }
                i++;
            }
        }
        cout << endl;
        presetArmConfsL.push_back(poseL);
        presetArmConfsR.push_back(poseR);
        ++count;
    }
    cout << "read over: " << endl;
    cout << "total configurations read: " << count << endl;
    pose_in_file.close();

    cout << "left arm configurations: " << endl;
    for (auto c : presetArmConfsL) {
        for (auto d : c) {
            cout << d << ", ";
        }
        cout << endl;
    }

    cout << "right arm configurations: " << endl;
    for (auto c : presetArmConfsL) {
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

/******************************************************************************/
// Update Waist State Value
double getWaistState() {
    struct timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
    int r;
    Somatic__MotorState *waist = NULL;
    while (waist == NULL) waist = getMotorMessage(waistChan);
    // Below line takes the average of the two waist motor readings
    // The second one is negative, hence the subtraction
    double waist_val = (waist->position->data[0] - waist->position->data[1]) / 2.0;
    return waist_val;
}

/******************************************************************************/
// Record data (the balanced pose) for offline COM analysis
// Output Munzir format w/o heading, x, y, z, qLWheel, qRWheel, qKinect:
// qBase (imu), qWaist, qTorso, qLArm0, ..., qLArm6, qRArm0, ..., qRArm6
void recordPoseData() {
    somatic_motor_update(&daemon_cx, &llwa);
    somatic_motor_update(&daemon_cx, &rlwa);

    double imu_val = getIMUState();
    cout << "imu value: " << imu_val << endl;
    pose_out_file << imu_val;

    double waist_val = getWaistState();
    cout << "waist value: " << waist_val << endl;
    pose_out_file << " " << waist_val;

    somatic_motor_update(&daemon_cx, &torso);
    cout << "torso value: " << torso.pos[0] << endl;
    pose_out_file << " " << torso.pos[0];

    cout << "left arm poses: ";
    for (int i = 0; i < 7; ++i) {
        cout << llwa.pos[i] << ", ";
        pose_out_file << " " << llwa.pos[i];
    }
    cout << endl;

    cout << "right arm poses: ";
    for (int i = 0; i < 7; ++i) {
        cout << rlwa.pos[i] << ", ";
        pose_out_file << " " << rlwa.pos[i];
    }
    cout << endl;
    pose_out_file << endl;
}

/******************************************************************************/
// Trigger to record data
// press d and enter to record / press x and enter to flag to ignore previous
// line
void *kbhit(void *) {
    char input;
    double kOffset = 0.05;
    while (true) {
        input = cin.get();
        pthread_mutex_lock(&mutex);
        if (input == 'd') {
            recordPoseData();
        }
        if (input == 'x') {
            outfile << "DELETE PREVIOUS LINE" << endl;
        }
        pthread_mutex_unlock(&mutex);
    }
}

/******************************************************************************/
// Reads the joystick data into global variables 'b' and 'x', b for button press
// and x for axes data
void readJoystick() {
    // Get the message and check output is OK.
    int r = 0;
    Somatic__Joystick *js_msg =
        SOMATIC_GET_LAST_UNPACK(r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan);
    if (!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

    // Save values from joystick message buttons and save them to b
    for (size_t i = 0; i < 10; i++)
        b[i] = js_msg->buttons->data[i] ? 1: 0;

    //copy over axes data
    memcpy(x, js_msg->axes->data, sizeof(x));

    // Free the joystick message
    somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
}

/******************************************************************************/
// Initialize imu, waist command/state channesl
// and torso and arms with somatic motors
// and joystick channels
void init() {

    // Initialize daemon with options
    somatic_d_opts_t dopt;
    memset(&dopt, 0, sizeof(dopt)); // zero intialize
    // TODO identifier needs to changed when finalized
    dopt.ident = "pose-full-ctrl";
    somatic_d_init(&daemon_cx, &dopt);

    // Initialize IMU Data
    initIMU(daemon_cx, imuChan);

    // Initialze the waist channels
    somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
    somatic_d_channel_open(&daemon_cx, &waistCmdChan, "waistd-cmd", NULL);

    // Initialize the torso and arms module
    initTorso(daemon_cx, torso);
    initArm(daemon_cx, llwa, "llwa");
    initArm(daemon_cx, rlwa, "rlwa");

    // Initialize the joystick channel
    int r = ach_open(&js_chan, "joystick-data", NULL);
    aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n",
            ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

    // Send a message; set the even code and the priority
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
                    SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

    // Create a thread to wait for user input to begin balancing
    pthread_t kbhitThread;
    pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/******************************************************************************/
// Set Torso Target
void resetTorsoTarget() {
    somatic_motor_update(&daemon_cx, &torso);
    torso_pos_target = torso.pos[0];
}

/******************************************************************************/
// Set Left Arm Target
void resetLlwaTarget() {
    for (int i = 0; i < 7; ++i) {
        llwa_pos_target[i] = llwa.pos[i];
    }
}

/******************************************************************************/
// Set Right Arm Target
void resetRlwaTarget() {
    for (int i = 0; i < 7; ++i) {
        rlwa_pos_target[i] = rlwa.pos[i];
    }
}

/******************************************************************************/
// Set initial postion targets for toros and arms to existing positions
void init_pos_target() {
    somatic_motor_update(&daemon_cx, &llwa);
    somatic_motor_update(&daemon_cx, &rlwa);

    resetTorsoTarget();
    resetLlwaTarget();
    resetRlwaTarget();
}

/******************************************************************************/
// Stop All Movements
void haltMovement() {
    cout << "halting movements" << endl;
    //// Halt Waist
    //static Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
    //somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
    //somatic_metadata_set_time_now(waistDaemonCmd->meta);
    //somatic_metadata_set_until_duration(waistDeaemonCmd->meta, 0.1);
    //SOMATIC_PACK_SEND(&waistCmdChan, somatic__waist_cmd, waistDaemonCmd);

    // Halt Torso
    double dq [] = {0,0};
    somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, dq, 1, NULL);
    somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);

    // Halt Arm
    haltArm(daemon_cx, llwa);
    haltArm(daemon_cx, rlwa);
}

/******************************************************************************/
// check if one of the numerical buttons is being pressed
bool buttonPressed() {
    // if we are logging
    if (b[9] == 1) {
        return true;
    }

    bool command_btn = ((b[0] == 1) || (b[1] == 1) || (b[2] == 1) || (b[3] == 1));

    bool part_btn = ((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1) || (x[1] > 0.9));

    // valid waist command
    if (((x[5] < -0.9) || (x[5] > 0.9)) && command_btn) {
        return true;
    }

    // arm, shoulder, & torso configs
    if (part_btn && command_btn) {
        return true;
    }

    return false;
}

/******************************************************************************/
// Updates Arms Target
void updateArmTarget(double targetPose[], vector<double> configPose) {
    for (int i = 0; i < 7; ++i) {
        targetPose[i] = configPose[i];
    }
}

/******************************************************************************/
// Print current arm position
void printArmPos(double pos[], string dir) {
    cout << "new " << dir << " arm target";
    for (int i = 0; i < 6; ++i) {
        cout << pos[i] << ", ";
    }
    // Last value has no common at the end
    cout << pos[6];

    cout << endl;
}

/******************************************************************************/
// TODO
// Handles Arm Configurations
void controlArms() {

    // Left Arm

    // button 5 & 4 for reset current target
    if ((b[6] == 1) && (b[3] == 1)) {
        resetLlwaTarget();
    }

    //button 5 & 2 for reset to default position
    if ((b[6] == 1) && (b[1] == 1)) {
        if (!llwa_mv && input_end) {
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

    // TODO I believe below line is incorrect description
    // buttons 1 and 3 are for clockwise/counterclockwise position change
    // buttons 5 & 1 and 5 & 3 are for forward/backward position movements
    if (((b[6] == 1) && (b[0] == 1)) || ((b[6] == 1) && (b[2] == 1))) {
        if (!llwa_mv && input_end) {
            cout << "new left arm config" << endl;
            // if not moving currently, update reset and move flags
            llwa_reset = true;
            llwa_mv = true;
            // we only update our target if we have reach our previous target or
            // the direction is difference
            if (llwa_reached || (((b[0] == 1) && (llwa_dir == -1 )) || (( b[2] == 1) && (llwa_dir == 1)))) {
                // if reached previous location, decrease/increase target by
                // step
                if (b[0] == 1) {
                    llwa_dir = 1;
                } else {
                    llwa_dir = -1;
                }
                cout << "left arm config id: " << llwa_config_idx;
                cout << "size of preset config: " << presetArmConfsL.size();
                llwa_config_idx = (llwa_config_idx + llwa_dir) % presetArmConfsL.size();
                cout << "left arm config id: " << llwa_config_idx;
                updateArmTarget(llwa_pos_target, presetArmConfsL[llwa_config_idx]);
                printArmPos(llwa_pos_target, "left");
                llwa_reached = false;
            }
        }
        return;
    }

    // Right Arm

    // button 6 & 4 for reset curret target
    if ((b[7] == 1) && (b[3] == 1)) {
        resetRlwaTarget();
    }

    // button 6 & 2 for reset to default position
    if ((b[7] == 1) && (b[1] == 1)) {
        if (!rlwa_mv && input_end) {
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

    // TODO I believe below line is incorrect description
    // buttons 1 and 3 are for clockwise/counterclockwise position change
    // buttons 6 & 1 and 6 & 3 are for forward/backward position movements
    if (((b[7] == 1) && (b[0] == 1)) || ((b[7] == 1) && (b[2] == 1))) {
        if (!rlwa_mv && input_end) {
            cout << "new right arm config" << endl;
            // if not moving currently, update reset and move falgs
            rlwa_reset = true;
            rlwa_mv = true;

            if (rlwa_reached || (((b[0] == 1) && (rlwa_dir == -1)) || ((b[2] == 1) && (rlwa_dir == 1)))) {
                // if reached previous location, decrease/increase target by
                // step
                if (b[0] == 1) {
                    rlwa_dir = 1;
                } else {
                    rlwa_dir = -1;
                }
                cout << "right arm config id: " << rlwa_config_idx;
                cout << "size of preset config: " << (presetArmConfsR.size());
                rlwa_config_idx = (llwa_config_idx + llwa_dir) % presetArmConfsL.size();
                cout << "right arm config id: " << rlwa_config_idx;
                updateArmTarget(rlwa_pos_target, presetArmConfsR[rlwa_config_idx]);
                printArmPos(rlwa_pos_target, "right");
                rlwa_reached = false;
            }
        }
        return;
    }
}

/******************************************************************************/
// Handles Torso
void controlTorso() {

    // axis LS down & button 4 for reset current target
    if ((x[1] > 0.9) && (b[3] == 1)) {
        resetTorsoTarget();
    }

    // axis LS down & button 2 for reset to default position
    if ((x[1] > 0.9) && (b[1] == 1)) {
        if (!torso_mv && input_end) {
            // if not moving currently, update reset and move flags
            torso_reset = true;
            torso_mv = true;

            // update target
            torso_pos_target = torso_pos_default;
            cout << "new torso target: " << torso_pos_target << endl;

            // if we reached the previous destination, reset reached flag
            if (torso_reached) {
                torso_reached = false;
                torso_dir = 0;
            }
        }
        return;
    }

    // axis LS down & button 1 and axis LS down & button 3 are for forward/backward position movements
    if (((x[1] > 0.9) && (b[0] == 1)) || ((x[1] > 0.9) && (b[2] == 1))) {
        if (!torso_mv && input_end) {
            // if not moving currently, update reset and move flags
            torso_reset = true;
            torso_mv = true;

            if (torso_reached || (((b[0] == 1) && (torso_dir == -1)) || ((b[2] == 1) && (torso_dir == 1)))) {
                // if reached previous location, decrease/increase target by
                // step
                if (b[0] == 1) {
                    torso_dir = 1;
                } else {
                    torso_dir = -1;
                }
                torso_config_idx = (torso_config_idx + torso_dir) % presetTorsoConfs.size();
                torso_pos_target = resetTorsoConfs[torso_config_idx];
                cout << "new torso target: " << torso_pos_target << endl;
                torso_reached = false;
            }
        }
        return;
    }
}

/******************************************************************************/
// TODO test this code
// Handles Torso and Arms
void controlTorsoAndArms() {

    // axis LS down & button 5 & 6 & 4 for reset current target
    if ((x[1] > 0.9) && (b[4] == 1) && (b[5] == 1) && (b[3] == 1)) {
        resetTorsoTarget();
        resetLlwaTarget();
        resetRlwaTarget();
    }

    // axis LS down & button 5 & 6 & 2 for reset to default position
    if ((x[1] > 0.9) && (b[4] == 1) && (b[5] == 1) && (b[1] == 1)) {
        // TODO should these be ors or ands basically can the torso and arms be
        // separately flagged (||) or need to be flagged together (&&)
        if (!torso_mv && !llwa_mv && !rlwa_mv && input_end) {
            // if not moving currently, update reset and move flags
            torso_reset = true;
            torso_mv = true;
            llwa_reset = true;
            llwa_mv = true;
            rlwa_reset = true;
            rlwa_mv = true;

            // update target
            torso_pos_target = torso_pos_default;
            cout << "new torso target: " << torso_pos_target << endl;
            updateArmTarget(llwa_pos_target, llwa_pos_default);
            printArmPos(llwa_pos_target, "left");
            updateArmTarget(rlwa_pos_target, rlwa_pos_default);
            printArmPos(rlwa_pos_target, "right");

            // if we reached the previous destination, reset reached flag
            if (torso_reached && llwa_reached && rlwa_reached) {
                torso_reached = false;
                torso_dir = 0;
                llwa_reached = false;
                llwa_dir = 0;
                rlwa_reached = false;
                rlwa_dir = 0;
            }
        }
        return;
    }

    // axis LS down & button 5 & 6 & 1 and axis LS down & button 5 & 6 & 3 are for forward/backward position movements
    if (((x[1] > 0.9) && (b[4] == 1) && (b[5] == 1) && (b[0] == 1)) || ((x[1] > 0.9) && (b[4] == 1) && (b[5] == 1) && (b[2] == 1))) {
        // TODO should these be ors or ands basically can the torso and arms be
        // separately flagged (||) or need to be flagged together (&&)
        if (!torso_mv && !llwa_mv && !rlwa_mv && input_end) {
            // TODO why the below two couts
            cout << "new left arm config" << endl;
            cout << "new right arm config" << endl;
            // if not moving currently, update reset and move flags
            torso_reset = true;
            torso_mv = true;
            llwa_reset = true;
            llwa_mv = true;
            rlwa_reset = true;
            rlwa_mv = true;

            // we only update our target if we have reach our previous target or
            // the direction is difference
            // TODO update the if statement potentially and determine if all
            // (next three if statements) motors should go inside one if statement
            if (torso_reached || (((b[0] == 1) && (torso_dir == -1)) || ((b[2] == 1) && (torso_dir == 1)))) {
                // if reached previous location, decrease/increase target by
                // step
                if (b[0] == 1) {
                    torso_dir = 1;
                } else {
                    torso_dir = -1;
                }
                torso_config_idx = (torso_config_idx + torso_dir) % presetTorsoConfs.size();
                torso_pos_target = resetTorsoConfs[torso_config_idx];
                cout << "new torso target: " << torso_pos_target << endl;
                torso_reached = false;
            }

            if (llwa_reached || (((b[0] == 1) && (llwa_dir == -1 )) || (( b[2] == 1) && (llwa_dir == 1)))) {
                // if reached previous location, decrease/increase target by
                // step
                if (b[0] == 1) {
                    llwa_dir = 1;
                } else {
                    llwa_dir = -1;
                }
                cout << "left arm config id: " << llwa_config_idx;
                cout << "size of preset config: " << presetArmConfsL.size();
                llwa_config_idx = (llwa_config_idx + llwa_dir) % presetArmConfsL.size();
                cout << "left arm config id: " << llwa_config_idx;
                updateArmTarget(llwa_pos_target, presetArmConfsL[llwa_config_idx]);
                printArmPos(llwa_pos_target, "left");
                llwa_reached = false;
            }

            if (rlwa_reached || (((b[0] == 1) && (rlwa_dir == -1)) || ((b[2] == 1) && (rlwa_dir == 1)))) {
                // if reached previous location, decrease/increase target by
                // step
                if (b[0] == 1) {
                    rlwa_dir = 1;
                } else {
                    rlwa_dir = -1;
                }
                cout << "right arm config id: " << rlwa_config_idx;
                cout << "size of preset config: " << (presetArmConfsR.size());
                rlwa_config_idx = (rlwa_config_idx + rlwa_dir) % presetArmConfsR.size();
                cout << "right arm config id: " << rlwa_config_idx;
                updateArmTarget(rlwa_pos_target, presetArmConfsR[rlwa_config_idx]);
                printArmPos(rlwa_pos_target, "right");
                rlwa_reached = false;
            }
        }
        return;
    }
}

/******************************************************************************/
// Handles Joystick Commands
void processJS() {

    // TODO create a method that controls torso and arm
    controlTorsoAndArms();

    controlTorso();
    controlArms();

    // if no buttons are actively pressed we halt all movements
    if (buttonPressed()) {
        input_end = false;
    } else {
        pose_mv = (torso_mv || llwa_mv || rlwa_mv);
        if (pose_mv) { // released button while moving)
            hlt_mv = true;
            pose_mv = false;
            torso_mv = false;
            llwa_mv = false;
            rlwa_mv = false;
        }
        input_end = true;
    }
}

/******************************************************************************/
// Move Arm
void moveArm(somatic_motor_t &arm, double target[]) {
    double arm_target[7] = {0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 7; ++i) {
        arm_target[i] = target[i];
    }
    somatic_motor_cmd(&daemon_cx, &arm, POSITION, arm_target, 7, NULL);
}

/******************************************************************************/
// Check each body part and send motor reset/movement commands (other than
// waist)
void applyMove() {

    if (hlt_mv) {
        haltMovement();
        hlt_mv = false;
    }

    if (torso_reset) {
        somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
        torso_reset = false;
    }

    if (llwa_reset) {
        cout << "resetting left arm" << endl;
        somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
        llwa_reset = false;
    }

    if (torso_reset) {
        somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
        rlwa_reset = false;
    }

    usleep(1e5);

    if (torso_mv) {
        double torso_pos_array[1] = {torso_pos_target};
        somatic_motor_cmd(&daemon_cx, &torso, POSITION, torso_pos_array, 1, NULL);
    }

    if (llwa_mv) {
        somatic_motor_update(&daemon_cx, &llwa);
        moveArm(llwa, llwa_pos_target);
    }

    if (rlwa_mv) {
        somatic_motor_update(&daemon_cx, &rlwa);
        moveArm(rlwa, rlwa_pos_target);
    }
}

/******************************************************************************/
// Check the 7 motors (indexed 0 to 6) of the arm position and match with pose
// target
bool checkArm(double pos[7], double target[7], double del) {
    for (int i = 0; i < 7; ++i) {
        if (fabs(pos[i] - target[i]) >= del) {
            return false;
        }
    }
    return true;
}

/******************************************************************************/
// Update poses and stop movements if we have reached our goals
void poseUpdate() {

    somatic_motor_update(&daemon_cx, &torso);
    somatic_motor_update(&daemon_cx, &llwa);
    somatic_motor_update(&daemon_cx, &rlwa);

    // check torso configuration
    if ((fabs(torso.pos[0] - torso_pos_target) < POSE_TOL) && !torso_reached) {
        cout << "torso target reached" << endl;
        double dq[] = {0.0};
        somatic_motor_cmd(&deamon_cx, &torso, VELOCITY, dq, 1, NULL);
        somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
        torso_reached = true;
    }

    // check arm configuration
    if (checkArm(llwa.pos, llwa_pos_target, POSE_TOL) && !llwa_reached) {
        cout << "left armr target reached" << endl;
        llwa_reached = true;
    }

    if (checkArm(rlwa.pos, rlwa_pos_target, POSE_TOL) && !rlwa_reached) {
        cout << "right arm target reached" << endl;
        rlwa_reached = true;
    }
}

/******************************************************************************/
// Handles Somatic Signals
void run() {
    // get initial states
    init_pos_target();

    readPoseFile();

    // Unless an interrupt or terminate message is received, process the new
    // message
    while (!somatic_sig_received) {

        // Read the joystick data
        readJoystick();

        processJS();

        applyMove();

        poseUpdate();

        usleep(1e4);

    }

    // Send the stopping even
    somatic_d_even(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
            SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

    // close our write file
    pose_out_file.close();
}

/******************************************************************************/
// Close channels and daemons
void destroy() {

    // Halt the waist modules
    static Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
    somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);

    // Do we need meta data?
    somatic_metadata_set_time_now(waistDaemonCmd->meta);
    somatic_metadata_set_until_duration(waistDaemonCmd->meta, 0.1);

    ach_status_t r = SOMATIC_PACK_SEND(&waistCmdCChan, somatic__waist_cmd, waistDaemonCmd);
    if (ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", ach_result_to_string(r));
    somatic_d_channel_close(&daemon_cx, &waistChan);
    somatic_d_channel_close(&daemon_cx, &waistCmdChan);

    // Halt the Schunk modules
    somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
    somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);

    // Close imu state channels
    somatic_d_channel_close(&daemon_cx, &imuChan);

    // Destroy the daemon resources
    somatic_d_destroy(&daemon_cx);

    cout << "Destroyed and halted" << endl;
}

/******************************************************************************/
// Main Method
int main() {
    init();
    run();
    destroy();

    return 0;
}
