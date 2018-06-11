/**
 * @file 01-balance.cpp
 * @author Munzir Zafar
 * @date July 26, 2013
 * @brief This code implements the balancing with force-compensations along with basic joystick 
 * control mainly for the demo on July 31st, 2013 using the newly developed kore library.
 */

#include "helpers.h"

using namespace std;
using namespace dynamics;

/* ******************************************************************************************** */
// For logging purposes

struct LogState {
	
	// Read state and sensors
	double time;
	Vector3d com;
	double averagedTorque, torque;
	double amcLeft, amcRight;

	// Controller stuff
	Vector6d state, refState;
	double lastUleft, lastUright;

	/// Constructor
	LogState (double t, const Vector3d& c, double aTo, double to, double aL, double aR, 
	          const Vector6d& s, const Vector6d& rS, double lUl, double lUr) :
		time(t), com(c), averagedTorque(aTo), torque(to), amcLeft(aL), amcRight(aR), state(s), 
		refState(rS), lastUleft(lUl), lastUright(lUr) {}

	/// Print
	void print () {
		//       torques         currents        state        refstate       time
		printf("%lf\t%lf\t  %lf\t%lf\t%lf\t%lf\t  %lf\t%lf\t%lf\t %lf\t%lf\t%lf\t %lf\n", 
		       averagedTorque, torque, lastUleft, lastUright, krang->amc->cur[0], krang->amc->cur[1],  
		       state(0)*180.0/M_PI, state(2)*180.0/M_PI, state(4)*180.0/M_PI, refState(0)*180.0/M_PI, 
		       refState(2)*180.0/M_PI, refState(4)*180.0/M_PI, time);
	}
	
};

/// The vector of states
vector <LogState*> logStates;

/* ******************************************************************************************** */
// Offset values for FT sensing

Vector6d leftOffset; 
Vector6d leftWheelWrench;
Vector6d rightOffset;
Vector6d rightWheelWrench;
bool debugGlobal = false, logGlobal = true;

/* ******************************************************************************************** */
/// Computes the wrench on the wheels due to external force from the f/t sensors' data
void getExternalWrench (Vector6d& external) {

	// If the wrench sensed on left FT sensor is not high use it calculate wrench on the wheel
	if((krang->fts[Krang::LEFT]->lastExternal.topLeftCorner<3,1>().norm() > 7) || 
	   (krang->fts[Krang::LEFT]->lastExternal.bottomLeftCorner<3,1>().norm() > 0.4)) {
		computeWheelWrench(krang->fts[Krang::LEFT]->lastExternal, *robot, leftWheelWrench, true);
	}
	else leftWheelWrench = Vector6d::Zero();

	// If the wrench sensed on right FT sensor is not high use it calculate wrench on the wheel
	if((krang->fts[Krang::RIGHT]->lastExternal.topLeftCorner<3,1>().norm() > 7) || 
	   (krang->fts[Krang::RIGHT]->lastExternal.bottomLeftCorner<3,1>().norm() > 0.4)) {
		computeWheelWrench(krang->fts[Krang::RIGHT]->lastExternal, *robot, rightWheelWrench, false);
	}
	else rightWheelWrench = Vector6d::Zero();
			
	// Sum the wheel wrenches from the two f/t sensors
	external = leftWheelWrench + rightWheelWrench;
}

/* ******************************************************************************************** */
/// Computes the reference balancing angle from center of mass, total mass and the felt wrenches.
/// The idea is that we want to set the balancing angle so that the torque due to the external 
/// force/torques and those due to the mass of the robot cancel each other out. 
/// Let com_x be the x component of the desired com such that com_x * mass * gravity = external
/// torque. Now, given that we know com, we can compute its distance from the origin, and then
/// compute the z component. The atan2(x,z) is the desired angle.
void computeBalAngleRef(const Vector3d& com, double externalTorque, double& refImu) {

	// Compute the x component of the desired com
	static const double totalMass = 142.66;
	double com_x = externalTorque / (totalMass * 9.81);

	// Compute the z component of the desired com by first computing the norm (which is fixed)
	// and then computing the z from x component
	double normSq = com(0) * com(0) + com(2) * com(2);
	double com_z = sqrt(normSq - com_x * com_x);

	// Compute the expected balancing angle	
	refImu = atan2(-com_x, com_z);
}

/* ********************************************************************************************* */
// The preset arm configurations: forward, thriller, goodJacobian
double presetArmConfs [][7] = {
	{  1.102, -0.589,  0.000, -1.339,  0.000, -0.959,  0.000},
	{ -1.102,  0.589,  0.000,  1.339,  0.000,  0.959,  0.000},
	{  0.843,  1.680,  0.000, -1.368,  0.000, -1.794,  0.000},
	{ -0.843, -1.680,  0.000,  1.368,  0.000,  1.794,  0.000},
	{  1.400, -1.000,  0.000, -0.800,  0.000, -0.800,  0.000}, // Fix this?
	{ -1.400,  1.000,  0.000,  0.800,  0.000,  0.800,  0.000},
	{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
	{  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
};

/* ********************************************************************************************* */
/// Controls the arms
void controlArms () {

	// Return if the x[3] is being used for robotiq hands
	if(fabs(x[3]) > 0.2) return;

	// Check if one of the preset configurations are requested by pressing 9 and
	// any of the buttons from 1 to 4 at the same time
	if((b[4] == 1) && (b[5] == 1) && (b[6] == 1) && (b[7] == 1)) {

		// Check if the button is pressed for the arm configuration is pressed, if so send pos commands
		bool noConfs = true;
		for(size_t i = 0; i < 4; i++) {
			if(b[i] == 1) {
				somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT], POSITION, presetArmConfs[2*i], 7, NULL);
				somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT], POSITION, presetArmConfs[2*i+1], 7, NULL);
				noConfs = false; 
				return;
			}
		}
		
		// If nothing is pressed, stop the arms
		if(noConfs) {
			double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::LEFT], VELOCITY, dq, 7, NULL);
			somatic_motor_cmd(&daemon_cx, krang->arms[Krang::RIGHT], VELOCITY, dq, 7, NULL);
			return;
		}
	}
	
	// Check the b for each arm and apply velocities accordingly
	// For left: 4 or 6, for right: 5 or 7, lower arm button is smaller (4 or 5)
	somatic_motor_t* arm [] = {krang->arms[Krang::LEFT], krang->arms[Krang::RIGHT]};
	for(size_t arm_idx = 0; arm_idx < 2; arm_idx++) {

		// Initialize the input
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Change the input based on the lower or higher button input
		bool inputSet = true;
		size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
		if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
		else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));
		else inputSet = false;
		
		// Set the input for this arm
		if(inputSet) somatic_motor_cmd(&daemon_cx, arm[arm_idx], VELOCITY, dq, 7, NULL);
	}
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the waist module
void controlWaist() {

	// Set the mode we want to send to the waist daemon
	Somatic__WaistMode waistMode;
	if(x[5] < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(x[5] > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
	else waistMode = SOMATIC__WAIST_MODE__STOP;

	// Send message to the krang-waist daemon
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
	                        ach_result_to_string(static_cast<ach_status_t>(r)));
}

/* ********************************************************************************************* */
/// Control the Robotiq hands - button 5 or 6 sets left/right, axis 3 sets the direction
/// TODO: Find a better joystick to command mapping. The results are not very intuitive.
void controlRobotiq() {

	// Return if x[2] is being used for the L7/R7 motors
	if(fabs(x[2]) > 0.2) return;

	// Skip some loops
	static int counter = 0;
	if(counter++ % 20 != 0) return;

	// For each hand, check if the button is pressed and if so give the command
	for(size_t i = 0; i < 2; i++) {

		// Turn the axis value between [-1, 1], to [0,255] - if 0, do nothing.
		int val = fmax(0, fmin(255, ((x[3] + 1.0) / 2.0) * 256));

		// Write the command line command and execute it
		char buf [256];
		if((b[4 + i] == 1) && (fabs(x[3]) > 0.05))  {
			sprintf(buf, "robotiq_cmd b gp%x -c %cgripper-cmd", val, i ? 'r' : 'l');
			printf("'%s'\n", buf);
			system(buf);
		}
	}
}

/* ******************************************************************************************** */
/// The continuous control loop which has 4 state variables, {x, x., psi, psi.}, where
/// x is for the position along the heading direction and psi is the heading angle. We create
/// reference x and psi values from the joystick and follow them with pd control.
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initially the reference position and velocities are zero (don't move!) (and error!)
	// Initializing here helps to print logs of the previous state
	Vector6d refState = Vector6d::Zero(), state = Vector6d::Zero(), error = Vector6d::Zero();

	// Read the FT sensor wrenches, shift them on the wheel axis and display
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	double time = 0.0;
	Vector6d externalWrench;
	Vector3d com;
	
	// Initialize the running history
	const size_t historySize = 60;
	vector <double> torqueHistory;
	for(size_t i = 0; i < historySize; i++) torqueHistory.push_back(0.0);
	
	// Initialize FT offset stuff
	Vector6d leftFTData, rightFTData, temp;
	leftFTData << 0,0,0,0,0,0;
	rightFTData << 0,0,0,0,0,0; 
	size_t leftFTIter = 0, rightFTIter = 0;
	
	// Continue processing data until stop received
	double js_forw = 0.0, js_spin = 0.0, averagedTorque = 0.0, lastUleft = 0.0, lastUright = 0.0;
	size_t mode4iter = 0, mode4iterLimit = 100;
	size_t lastMode = MODE;
	while(!somatic_sig_received) {

		bool debug = (c_++ % 20 == 0);
		debugGlobal = debug;
		if(debug) cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << endl;

		// =======================================================================
		// Get inputs: time, joint states, joystick and external forces

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	
		t_prev = t_now;
		time += dt;

		// Get the joystick input for the js_forw and js_spin axes (to set the gains)
		bool gotInput = false;
		while(!gotInput) gotInput = getJoystickInput(js_forw, js_spin);
		if(debug) cout << "js_forw: " << js_forw << ", js_spin: " << js_spin << endl;

		// Get the current state and ask the user if they want to start
		getState(state, dt, &com);
		if(debug) cout << "\nstate: " << state.transpose() << endl;
		if(debug) cout << "com: " << com.transpose() << endl;

		// Get the wrench on the wheel due to external force
		getExternalWrench(externalWrench);
		if(debug) cout << "tangible torque: " << externalWrench(4) << endl;
		
		// Perform a running average on the felt torque on the wheel by adding the index 
		// and averaging the data again
		averagedTorque = 0.0;
		torqueHistory[c_ % historySize] = externalWrench(4);
		for(size_t i = 0; i < historySize; i++) averagedTorque += torqueHistory[i];
		averagedTorque /= historySize;
		
		// =======================================================================
		// Compute ref state: (1) joystick, (2) running average of external, (3) ref angle

		// Update the reference values for the position and spin
		// NOTE Don't print refState here, the theta ref is going to be overridden
		updateReference(js_forw, js_spin, dt, refState);
	
		// Cancel any position built up in previous mode
		if(lastMode != MODE) {
			refState(2) = state(2), refState(4) = state(4);
			lastMode = MODE;
		}
	
		// Compute the balancing angle reference using the center of mass, total mass and felt wrench.
		if(complyTorque) computeBalAngleRef(com, averagedTorque, refState(0));
		if(debug) cout << "refState: " << refState.transpose() << endl;

		// =======================================================================
		// Apply control: compute error, threshold and send current

		// Compute the error term between reference and current, and weight with gains (spin separate)
		if(debug) cout << "K: " << K.transpose() << endl;
		error = state - refState;

		// If we are in the sit down mode, over write the reference
		if(MODE == 3) {
			static const double limit = ((-103.0 / 180.0) * M_PI);
			if(krang->imu < limit) {
				printf("imu (%lf) < limit (%lf): changing to mode 1\n", krang->imu, limit);
				MODE = 1;
				K = K_ground;

			}
			else error(0) = krang->imu - limit;
		}

		// COM error correction in balHigh mode
		else if(MODE == 5) {
			error(0) -= 0.005;	
		}
		if(debug) cout << "error: " << error.transpose() << ", imu: " << krang->imu / M_PI * 180.0 << endl;

		// Compute the current
		double u = K.topLeftCorner<4,1>().dot(error.topLeftCorner<4,1>());
		double u_spin =  -K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
		u_spin = max(-15.0, min(15.0, u_spin));
    	
		// Compute the input for left and right wheels
		double input [2] = {u + u_spin, u - u_spin};
		input[0] = max(-49.0, min(49.0, input[0]));
		input[1] = max(-49.0, min(49.0, input[1]));
		if(debug) printf("u: %lf, u_spin: %lf\n", u, u_spin);
		lastUleft = input[0], lastUright = input[1];
		
		// Set the motor velocities
		if(start) {
			if(debug) cout << "Started..." << endl;
			somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
		}
	
		// ==========================================================================
		// Switching modes based on certain events
		
		// Stand/sit if button 10 is pressed
		static bool b9Prev = 0; // To store the value of button 9 in last iteration
		
		if(b9Prev == 0 && b[9] == 1) {

			// If in ground mode and state error is not high stand up
			if(MODE == 1) {
				if(state(0) < 0.0 && error(0) > -10.0*M_PI/180.0)	{
					printf("\n\n\nMode 2\n\n\n");
					K = K_stand;
					MODE = 2;	
				}	else {
					printf("\n\n\nCan't stand up, balancing error is too high!\n\n\n");
				}
			}
			
			// If in balLow mode and waist is not too high, sit down
			else if(MODE == 4) {
				if((krang->waist->pos[0] - krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
					printf("\n\n\nMode 3\n\n\n");
					K = K_sit;
					MODE = 3;	
				} else {
					printf("\n\n\nCan't sit down, Waist is too high!\n\n\n");
				}
			}
		}
	
		// Store the value of button 10 in the last iteration
		b9Prev = b[9];

		// If in standing up mode check if the balancing angle is reached and stayed, if so switch to balLow mode
		if(MODE == 2) {
			if(fabs(state(0)) < 0.01) mode4iter++;
			// Change to mode 4 (balance low) if stood up enough
			if(mode4iter > mode4iterLimit) {
				MODE = 4;
				mode4iter = 0;
				K = K_balLow;
			}
		}
		
		// Print the mode
		if(debug) printf("Mode : %d\n", MODE);
	
		// =======================================================================
		// Control the arms, waist torso and robotiq grippers based on the joystick input

		// Control the arms if necessary
		controlArms();

		// Control the torso
		double dq [] = {x[4] / 7.0};
		somatic_motor_cmd(&daemon_cx, krang->torso, VELOCITY, dq, 1, NULL);

		// Control the waist
		controlWaist();

		// Control the robotiq hands
		controlRobotiq();
		
		// ============================================================================
		// Logging 

		// Print the information about the last iteration (after reading effects of it from sensors)
		// NOTE: Constructor order is NOT the print order
		if(logGlobal) {
			logStates.push_back(new LogState(time, com, averagedTorque, externalWrench(4), krang->amc->cur[0],
			                                 krang->amc->cur[1], state, refState, lastUleft, lastUright));
		}

		// ==========================================================================
		// Quit if button 9 on the joystick is pressed, 

		if(b[8] == 1) break;
/*
// Compute the offsets of the FT sensor again if commanded
if(resetFT) {

if(debug)	cout << "Resetting FT" << endl;
// Accumulate data for LEft FT
if(getFT(daemon_cx, left_ft_chan, temp) && leftFTIter < 500)  {	leftFTData += temp;	leftFTIter++; }
if(getFT(daemon_cx, right_ft_chan, temp) && rightFTIter < 500) { rightFTData += temp;	rightFTIter++;}
		
// If done accumulating data compute the average
if(leftFTIter == 500 && rightFTIter == 500) {
leftFTData /= 500.0, rightFTData /= 500.0;
computeOffset(imu, (waist.pos[0]-waist.pos[1])/2.0, llwa, leftFTData, *robot, leftOffset, true);
computeOffset(imu, (waist.pos[0]-waist.pos[1])/2.0, rlwa, rightFTData, *robot, rightOffset, false);
leftFTData << 0,0,0,0,0,0; rightFTData << 0,0,0,0,0,0; 
leftFTIter = 0, rightFTIter = 0; 
resetFT = false;
}
} 
*/

	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Initialize the motor and daemons
void init() {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "01-balance";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	krang = new Krang::Hardware(Krang::Hardware::MODE_ALL_GRIPSCH, &daemon_cx, robot); 

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
	               ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
}

/* ******************************************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon structs.
void destroy() {

	cout << "destroying" << endl;

	// Stop motors, close motor/sensor channels and destroy motor objects
	delete krang;
		
	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	// Print the data
	printf("log states size: %lu\n", logStates.size());
	for(size_t i = 0; i < logStates.size(); i++) {
		logStates[i]->print();
		delete logStates[i];
	}
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	// Load the world and the robot
	DartLoader dl;
	world = dl.parseWorld("../../../experiments/common/scenes/01-World-RobotSchunkGrippers.urdf");
	assert((world != NULL) && "Could not find the world");
	robot = world->getSkeleton(0);

	// Read the gains from the command line
/*
  assert(argc >= 7 && "Where is my gains for th, x and spin?");
  K << atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]);
  jsFwdAmp = 0.3;
  jsSpinAmp = 0.4;
	
  cout << "K_bal: " << K_bal.transpose() << "\nPress enter: " << endl;
*/

	readGains();

	// Debug options from command line
	debugGlobal = 1; logGlobal = 0;
	if(argc == 8) { 
		if(argv[7][0]=='l') { debugGlobal = 0; logGlobal = 1;} 
		else if(argv[7][0] == 'd') {debugGlobal = 1; logGlobal = 0; } 
	} 

	getchar();

	// Initialize, run, destroy
	init();
	run();
	destroy();
	return 0;
}
