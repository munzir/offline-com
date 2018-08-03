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
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "initModules.h"
#include "motion.h"

// // Namespaces
using namespace std;

// // Defines
#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define getMotorMessage(x) (SOMATIC_WAIT_LAST_UNPACK(r, somatic__motor_state, \
    &protobuf_c_system_allocator, 1024, &x, &abstime))

// tolerance for the pose
// TODO Test lower tolerance value
#define POSE_TOL 0.01
//#define POSE_TOL 0.005

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
bool waist_mv = false;

bool llwa_reached = true;
bool rlwa_reached = true;
bool torso_reached = true;
bool waist_reached = true;

// -1 for backwards direction and 1 for positive direction in pose file
int llwa_dir = 0;
int rlwa_dir = 0;
int torso_dir = 0;
int waist_dir = 0;

bool llwa_reset = false;
bool rlwa_reset = false;
bool torso_reset = false;
bool waist_reset = false;

double llwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double rlwa_pos_target[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_target = 0.0;
// TODO this might be a dangerous default value
// Check below waist value again if waist control is ever implemented in this
// script
double waist_pos_target = 0.0;

vector<double> llwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
vector<double> rlwa_pos_default = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double torso_pos_default = 0.0;
// TODO this might be a dangerous default value
// Check below waist value again if waist control is ever implemented in this
// script
double waist_pos_default = 0.0;

// Index of the current pose in the read file
int llwa_config_idx = 0;
int rlwa_config_idx = 0;
int torso_config_idx = 0;
int waist_config_idx = 0;

vector<vector<double>> presetArmConfsL;
vector<vector<double>> presetArmConfsR;
vector<double> presetTorsoConfs;
vector<double> presetWaistConfs;

// TODO add extract filename from path functionality
// INPUT on below line (input and output file names)
string inputPoseFilename = "interposeTraj1-2.txt"
ifstream pose_in_file("../data/dataIn/poseTrajectoriesrfinalSet/" + inputPoseFilename + ".txt");
ofstream pose_out_file("../data/dataOut/" + inputPoseFilename + "out.txt", ios::app);

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/******************************************************************************/
// Update qBase (imu) state value
double getIMUState() {
    struct timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
    int r;
    double imu, imuSpeed;
    Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector,
            &protobuf_c_system_allocator, IMU_CHANNEL_SIZE, &imuChan, &abstime);
    assert((imu_msg != NULL) && "Imu message is faulty!");

    // Get the imu position and velocity value from the readings (note imu
    // mounted at 45 deg).
    static const double mountAngle = -0.7853981634;
    double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
    double _imu = atan2(newX, imu_msg->data[2]);

    // Free the unpacked message
    somatic__vector__free_unpacked(imu_msg, &protobuf_c_system_allocator);

    return _imu;
}

/******************************************************************************/
// Read and write poses to file
void readPoseFile() {
    int count = 0;
    string line;
    presetArmConfsL.push_back(llwa_pos_default);
    presetArmConfsR.push_back(rlwa_pos_default);
    presetTorsoConfs.push_back(torso_pos_default);
    presetWaistConfs.push_back(waist_pos_default);
    while (getline(pose_in_file, line)) {
        //cout << "new line: " << line << endl;
        stringstream lineStream(line);
        string strNum;
        double doubNum;
        vector<double> poseL;
        vector<double> poseR;
        int i = 0;
        //cout << "read number";
        while (getline(lineStream, strNum, ' ')) {
            if (!strNum.empty()) {
                istringstream convert(strNum);
                convert >> doubNum;
                //cout << doubNum;
                // TODO change indexes to reflect munzir format (eventually not
                // yet)
                if (i == 8) {
                    presetWaistConfs.push_back(doubNum);
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
        //cout << endl;
        presetArmConfsL.push_back(poseL);
        presetArmConfsR.push_back(poseR);
        ++count;
    }

    // Print out the file read for confirmation

    //cout << "read over" << endl;
    //cout << "total configurations read: " << count << endl;
    //pose_in_file.close();

    //cout << "waist configurations" << endl;
    //for (auto c : presetWaistConfs) {
    //    cout << c << endl;
    //}

    //cout << "torso configurations" << endl;
    //for (auto c : presetTorsoConfs) {
    //    cout << c << endl;
    //}

    //cout << "left arm configurations: " << endl;
    //for (auto c : presetArmConfsL) {
    //    for (auto d : c) {
    //        cout << d << ", ";
    //    }
    //    cout << endl;
    //}

    //cout << "right arm configurations: " << endl;
    //for (auto c : presetArmConfsL) {
    //    for (auto d : c) {
    //        cout << d << ", ";
    //    }
    //    cout << endl;
    //}
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
            pose_out_file << "DELETE PREVIOUS LINE" << endl;
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

    // TODO
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

    // buttons 1 or 2 or 3 or 4
    bool command_btn = ((b[0] == 1) || (b[1] == 1) || (b[2] == 1) || (b[3] == 1));

    // For full control
    // buttons 5 & 7 & 8
    bool part_btn = ((b[4] == 1) && (b[6] == 1) && (b[7] == 1));

    // For individual control
    //bool part_btn = ((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1) || (x[1] > 0.9));

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
                torso_pos_target = presetTorsoConfs[torso_config_idx];
                cout << "new torso target: " << torso_pos_target << endl;
                torso_reached = false;
            }
        }
        return;
    }
}

/******************************************************************************/
// Handles Torso and Arms
void controlTorsoAndArms() {

    // button 5 & 7 & 8 & 4 for reset current target
    if ((b[4] == 1) && (b[6] == 1) && (b[7] == 1) && (b[3] == 1)) {
        //resetWaistTarget();
        resetTorsoTarget();
        resetLlwaTarget();
        resetRlwaTarget();
    }

    // button 5 & 7 & 8 & 2 for reset to default position
    if ((b[4] == 1) && (b[6] == 1) && (b[7] == 1) && (b[1] == 1)) {
        if (!torso_mv && !llwa_mv && !rlwa_mv && input_end) {
            // if not moving currently, update reset and move flags
            waist_reset = true;
            waist_mv = true;
            torso_reset = true;
            torso_mv = true;
            llwa_reset = true;
            llwa_mv = true;
            rlwa_reset = true;
            rlwa_mv = true;

            // update target
            waist_pos_target = waist_pos_default;
            cout << "new waist targe: " << waist_pos_target << endl;
            torso_pos_target = torso_pos_default;
            cout << "new torso target: " << torso_pos_target << endl;
            updateArmTarget(llwa_pos_target, llwa_pos_default);
            printArmPos(llwa_pos_target, "left");
            updateArmTarget(rlwa_pos_target, rlwa_pos_default);
            printArmPos(rlwa_pos_target, "right");

            // if we reached the previous destination, reset reached flag
            if (torso_reached && llwa_reached && rlwa_reached) {
                waist_reached = false;
                waist_dir = 0;
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

    // button 5 & 7 & 8 & 3 and button 5 & 7 & 8 & 1 are for forward/backward position movements
    if (((b[4] == 1) && (b[6] == 1) && (b[7] == 1) && (b[2] == 1)) || ((b[4] == 1) && (b[6] == 1) && (b[7] == 1) && (b[0] == 1))) {
        if (!torso_mv && !llwa_mv && !rlwa_mv && input_end) {
            // if not moving currently, update reset and move flags
            waist_reset = true;
            waist_mv = true;
            torso_reset = true;
            torso_mv = true;
            llwa_reset = true;
            llwa_mv = true;
            rlwa_reset = true;
            rlwa_mv = true;

            // we only update our target if we have reach our previous target or
            // the direction is difference
            if ((torso_reached && llwa_reached && rlwa_reached) || (((b[2] == 1) && ((torso_dir == -1) && (llwa_dir == -1) && (rlwa_dir == -1))) || ((b[0] == 1) && ((torso_dir == 1) && (llwa_dir == 1) && (rlwa_dir == 1))))) {
                // if reached previous location, decrease/increase target by
                // step
                if (b[2] == 1) {
                    waist_dir = 1;
                    torso_dir = 1;
                    llwa_dir = 1;
                    rlwa_dir = 1;
                } else {
                    waist_dir = -1;
                    torso_dir = -1;
                    llwa_dir = -1;
                    rlwa_dir = -1;
                }


                waist_config_idx = (waist_config_idx + waist_dir) % presetWaistConfs.size();
                waist_pos_target = presetWaistConfs[waist_config_idx];
                cout << "new waist target: " << waist_pos_target << endl;
                waist_reached = false;

                torso_config_idx = (torso_config_idx + torso_dir) % presetTorsoConfs.size();
                torso_pos_target = presetTorsoConfs[torso_config_idx];
                cout << "new torso target: " << torso_pos_target << endl;
                torso_reached = false;

                //cout << "left arm config id: " << llwa_config_idx;
                //cout << "size of preset config: " << presetArmConfsL.size();
                llwa_config_idx = (llwa_config_idx + llwa_dir) % presetArmConfsL.size();
                //cout << "left arm config id: " << llwa_config_idx;
                updateArmTarget(llwa_pos_target, presetArmConfsL[llwa_config_idx]);
                printArmPos(llwa_pos_target, "left");
                llwa_reached = false;

                //cout << "right arm config id: " << rlwa_config_idx;
                //cout << "size of preset config: " << (presetArmConfsR.size());
                rlwa_config_idx = (rlwa_config_idx + rlwa_dir) % presetArmConfsR.size();
                //cout << "right arm config id: " << rlwa_config_idx;
                updateArmTarget(rlwa_pos_target, presetArmConfsR[rlwa_config_idx]);
                printArmPos(rlwa_pos_target, "right");

                rlwa_reached = false;
				cout << "Config IDs: W: " << waist_config_idx << ", T: " << torso_config_idx << ", L: " << llwa_config_idx << ", R:" << rlwa_config_idx << endl;
            }
        }
        return;
    }
}

/******************************************************************************/
// Handles Joystick Commands
void processJS() {

    controlTorsoAndArms();

    // if no buttons are actively pressed we halt all movements
    if (buttonPressed()) {
        input_end = false;
    } else {
        pose_mv = (torso_mv || llwa_mv || rlwa_mv);
        if (pose_mv) { // released button while moving)
            hlt_mv = true;
            pose_mv = false;
            waist_mv = false;
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

    // For full body movements
    if (torso_reset && llwa_reset && rlwa_reset) {

        //cout << "resetting torso" << endl;
        //cout << "resetting left arm" << endl;
        //cout << "resetting right arm" << endl;

        somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
        torso_reset = false;

        somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
        llwa_reset = false;

        somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
        rlwa_reset = false;
    }

    usleep(1e5);

    if (torso_mv && llwa_mv && rlwa_mv) {
        double torso_pos_array[1] = {torso_pos_target};
        somatic_motor_cmd(&daemon_cx, &torso, POSITION, torso_pos_array, 1, NULL);
        somatic_motor_update(&daemon_cx, &llwa);
        moveArm(llwa, llwa_pos_target);
        somatic_motor_update(&daemon_cx, &rlwa);
        moveArm(rlwa, rlwa_pos_target);
    }

    // For segmented movements
    //if (torso_reset) {
    //    somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 1, NULL);
    //    torso_reset = false;
    //}

    //if (llwa_reset) {
    //    cout << "resetting left arm" << endl;
    //    somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    //    llwa_reset = false;
    //}

    //if (rlwa_reset) {
    //    cout << "resetting right arm" << endl;
    //    somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 7, NULL);
    //    rlwa_reset = false;
    //}

    //usleep(1e5);

    //if (torso_mv) {
    //    double torso_pos_array[1] = {torso_pos_target};
    //    somatic_motor_cmd(&daemon_cx, &torso, POSITION, torso_pos_array, 1, NULL);
    //}

    //if (llwa_mv) {
    //    somatic_motor_update(&daemon_cx, &llwa);
    //    moveArm(llwa, llwa_pos_target);
    //}

    //if (rlwa_mv) {
    //    somatic_motor_update(&daemon_cx, &rlwa);
    //    moveArm(rlwa, rlwa_pos_target);
    //}
}

/******************************************************************************/
// Check the 7 motors (indexed 0 to 6) of the arm position and match with pose
// target
bool checkArm(double pos[7], double target[7], double tol) {
    for (int i = 0; i < 7; ++i) {
        if (fabs(pos[i] - target[i]) > tol) {
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

    // check full body configuration
    if (((fabs(torso.pos[0] - torso_pos_target) <= POSE_TOL) && !torso_reached) && (checkArm(llwa.pos, llwa_pos_target, POSE_TOL) && !llwa_reached) && (checkArm(rlwa.pos, rlwa_pos_target, POSE_TOL) && !rlwa_reached)) {

        // waist
        waist_reached = true;

        // torso
        double dq[] = {0.0};
        somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, dq, 1, NULL);
        somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
        torso_reached = true;

        // left arm
        llwa_reached = true;

        // right arm
        rlwa_reached = true;

        //cout << "torso target reached" << endl;
        //cout << "left arm target reached" << endl;
        //cout << "right arm target reached" << endl;

        cout << "********************************" << endl;
		cout << "****** INTERPOSE REACHED *******" << endl;
        cout << "********************************" << endl;
    }


    // Below checks for segemented body individually
    // check torso configuration
    //if ((fabs(torso.pos[0] - torso_pos_target) <= POSE_TOL) && !torso_reached) {
    //    cout << "torso target reached" << endl;
    //    double dq[] = {0.0};
    //    somatic_motor_cmd(&daemon_cx, &torso, VELOCITY, dq, 1, NULL);
    //    somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);
    //    torso_reached = true;
    //}

    //// check left arm configuration
    //if (checkArm(llwa.pos, llwa_pos_target, POSE_TOL) && !llwa_reached) {
    //    cout << "left arm target reached" << endl;
    //    llwa_reached = true;
    //}

    //// check right arm configuration
    //if (checkArm(rlwa.pos, rlwa_pos_target, POSE_TOL) && !rlwa_reached) {
    //    cout << "right arm target reached" << endl;
    //    rlwa_reached = true;
    //}
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
    somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
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

    ach_status_t r = SOMATIC_PACK_SEND(&waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
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
