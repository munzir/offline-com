/*
 * @file waist-control.cpp
 * @author Areeb Mehmood
 * @date August 3, 2018
 * @brief This file controls waist records data
 */

// // Includes
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/time.h>

#include "initModules.h"
#include "motion.h"

// // Namespaces
using namespace std;

// // Defines
#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define getMotorMessage(x) (SOMATIC_WAIT_LAST_UNPACK(r, somatic__motor_state, \
    &protobuf_c_system_allocator, 1024, &x, &abstime))

// // Global Stuff
// Init somatic daemon
somatic_d_t daemon_cx;

// Init ACH channels
ach_channel_t js_chan;      // the state channel for the joystick module
ach_channel_t waistChan;    // the state channel for the waist module
ach_channel_t waistCmdChan; // the state channel for the waist module //TODO why same description? Should it be 'command channel' instead

// Init somatic motor objects
somatic_motor_t llwa, rlwa, torso;

// Why do we need to allocate memory for waist command but not others?
Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();  // The waist command

// Init controller signals, b for buttons and x for axes
char b[10];
double x[6];

ofstream q_waist_out_file;
ofstream dq_waist_out_file;
ofstream currsom_waist_out_file;
ofstream currcmd_waist_out_file;
ofstream time_out_file;

long startTime;
double currTime;
struct timeval tv;

string inputLine;

struct waistStruct {
	double qWaist;
	double dqWaist;
	double currSomWaist;
};

waistStruct waistDataStruct;

/******************************************************************************/
// Read joystick data
void readJoystick() {
    // Get the message and check output is OK.
    int r = 0;
    Somatic__Joystick *js_msg =
        SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, 4096, &js_chan );
    if (!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return;

    // Save values from joystick message buttons and save them to be
    for (size_t i = 0; i < 10; i++)
        b[i] = js_msg->buttons->data[i] ? 1 : 0;

    // copy over axes data
    memcpy(x, js_msg->axes->data, sizeof(x));

    // Free the joystick message
    somatic__joystick__free_unpacked(js_msg, &protobuf_c_system_allocator);
}

/******************************************************************************/
void getWaistState() {
    struct timespec currTime;
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
    int r;

	// Get waist state
	Somatic__MotorState *waist = NULL;
	while (waist == NULL) waist = getMotorMessage(waistChan);

	// Get data from waist and put it into struct
	waistDataStruct.qWaist = (waist->position->data[0] - waist->position->data[1]) /2.0;
	waistDataStruct.dqWaist = (waist->velocity->data[0]);
	waistDataStruct.currSomWaist = (waist->current->data[0]);
}

/******************************************************************************/
unsigned long getTime() {
	gettimeofday(&tv, NULL);
	unsigned long ret = tv.tv_usec;
	ret /= 1000;
	ret += (tv.tv_sec*1000);
	return ret;
}

/******************************************************************************/
void recordWaistData() {
	// Record all positions including waist, torso and arms
	somatic_motor_update(&daemon_cx, &llwa);
	somatic_motor_update(&daemon_cx, &rlwa);
	somatic_motor_update(&daemon_cx, &torso);

	q_waist_out_file << waistDataStruct.qWaist << " " << torso.pos[0];
	for (int i = 0; i < 7; ++i) {
		q_waist_out_file << " " << llwa.pos[i];
	}
	for (int i = 0; i < 7; ++i) {
		q_waist_out_file << " " << rlwa.pos[i];
	}
	q_waist_out_file << endl;

	// Record waist speed
	dq_waist_out_file << waistDataStruct.dqWaist << endl;

	// Record System waist current from somatic
	currsom_waist_out_file << waistDataStruct.currSomWaist << endl;

	// Record Commanded waist current from output log
	ifstream myfile("/home/munzir/areeb/waist_log.txt", ios::in);
	if (myfile.is_open()) {
		getline(myfile, inputLine);
		currcmd_waist_out_file << inputLine << endl;
	}
	else {
		cout << "WAIST LOG FILE NOT OPEN" << endl;
	}

	// Record time
	currTime = (getTime() - startTime)/1000.0;

	time_out_file << currTime << endl;
	
	cout << "Time: " << currTime << endl
		<< "q: " << waistDataStruct.qWaist << " dq: " << waistDataStruct.dqWaist << endl
		<< "logCurr: " << inputLine << " cmdCurr: " << waistDataStruct.currSomWaist << endl << endl;
}


/******************************************************************************/
void controlWaist() {
	Somatic__WaistMode waistMode;
	if (x[5] < -0.9) {
      waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
  		getWaistState();		
			cout << waistDataStruct.qWaist << endl; //i
	} else if (x[5] > 0.9) {
      waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
 			getWaistState(); 
			cout << waistDataStruct.qWaist << endl;
	} else {
      waistMode = SOMATIC__WAIST_MODE__STOP;
  }

	// Send message
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	int r = SOMATIC_PACK_SEND(&waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n",
		ach_result_to_string(static_cast<ach_status_t>(r)));

    // Record data only if waist moved and button 6 pressed
    if ((x[5] < -0.9 || x[5] > 0.9) && (b[5] == 1)) {
        getWaistState();
        recordWaistData();
    }

}

/******************************************************************************/
void run() {
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
		SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	startTime = getTime();

	cout << "Ready" << endl;

	while (!somatic_sig_received) {

		readJoystick();

		controlWaist();

		//recordWaistData(); //continue to record data until program is stopped

		// Free buffers allocated during this cycle
		aa_mem_region_release(&daemon_cx.memreg);
		usleep(1e4);
	}

	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
		SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/******************************************************************************/
void init() {
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt));
	dopt.ident = "waist_control";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n",
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	somatic_d_channel_open(&daemon_cx, &waistChan, "waist-state", NULL);
	somatic_d_channel_open(&daemon_cx, &waistCmdChan, "waistd-cmd", NULL);

	// Initialize the arm and torso channels for reading their poses
	initArm(daemon_cx, llwa, "llwa");
	initArm(daemon_cx, rlwa, "rlwa");
	initTorso(daemon_cx, torso);

	// Initialize waist channel
	somatic_d_channel_open(&daemon_cx, &waistCmdChan, "waistd-cmd", NULL);

}

/******************************************************************************/
void destroy() {
	// Halt the waist modules
	static Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
	somatic_waist_cmd_set(waistDaemonCmd, SOMATIC__WAIST_MODE__STOP);
	somatic_metadata_set_time_now(waistDaemonCmd->meta);
	somatic_metadata_set_until_duration(waistDaemonCmd->meta, 0.1);
	ach_status_t r = SOMATIC_PACK_SEND(&waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", ach_result_to_string(r));
	somatic_d_channel_close(&daemon_cx, &waistChan);
	somatic_d_channel_close(&daemon_cx, &waistCmdChan);


	// Halt the arms and torso
	somatic_motor_cmd(&daemon_cx, &llwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &rlwa, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 7, NULL);
	somatic_motor_cmd(&daemon_cx, &torso, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 1, NULL);

	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);
}

/******************************************************************************/
int main(int argc, char *argv[]) {

    // INPUT on below line (input and output file names)
    // poseNum is the pose at which the waist is being controlled
    string poseNum = argv[1];

	string qWaistOutFilename = "../data/dataOut/waistData/" + poseNum + "qWaistOut.txt";
	string dqWaistOutFilename = "../data/dataOut/waistData/" + poseNum + "dqWaistOut.txt";
	string currSomWaistOutFilename = "../data/dataOut/waistData/" + poseNum + "currSomWaistOut.txt";
	string currCmdWaistOutFilename = "../data/dataOut/waistData/" + poseNum + "currCmdWaistOut.txt";
	string timeOutFilename = "../data/dataOut/waistData/" + poseNum + "timeWaistOut.txt";

	// Create out files with numbers given in execution statement
	q_waist_out_file.open(qWaistOutFilename, ios::app);
	dq_waist_out_file.open(dqWaistOutFilename, ios::app);
	currsom_waist_out_file.open(currSomWaistOutFilename, ios::app);
	currcmd_waist_out_file.open(currCmdWaistOutFilename, ios::app);
	time_out_file.open(timeOutFilename, ios::app);

    // Run controller and recording code
	init();
	run();
	destroy();

	return 0;
}
