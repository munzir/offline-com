# offline-com
This is the repo for Krang pose control files to be used primarily in offline center of mass and waist data collection. However, since these files control and set the pose based on position for all the joints except qBase and qWaist and velocity for qWaist, they can be used for a variety of applications.

## pose-full-control.cpp
A script that allows the robot to track a trajectory of poses from a file with
control using a joystick and can output current poses of the robot in a file.

### Dependencies
- TODO

### Build and Run
1: Enter the project directory

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./pose-full-control 3-4

### Input(s)
The input is a pose file in Munzir's format.
As of right now the pose files are hardcoded for trajectories around balanced
poses, named `interposeTraj{X-X+1}.txt`. The command line argument needed is
`X-X+1`, where X is a number from 1 to 527 (the range of trajectory files).
In addition the first and the last poses of the files are balanced and the waist
is moved independently during the trajectory.

### Commands
A joystick is used for the control of the poses. A series of buttons must be
pressed for the rest of the commands to work. This flag is specified below.

    flag : button 5 & 7 & 8
    // TODO: Understand loopback of poses in file/what to do with it
    button 1 : move to the previous pose in the file
    button 2 : reset to default position
    button 3 : move to the next pose in the file
    button 4 : reset current target
    pd       : record pose data
    px       : print reminder to ignore previous line in output file

### Output(s)
The output of this script is the pose data taken (qBase (imu), qWaist, qTorso,
qLArm0, ..., qLArm6, qRArm0, ..., qRArm6) in an output file.

## waist-control.cpp
A script that allows the robot to just move its waist position using a joystick
as input and also allows the recording of the pose, waist angle velocity
(dqWaist), waist current from both somatic (currSomWaist) and the current
command sent (currCmdWaist), and time.

### Dependencies
Same as `pose-full-control.cpp`

### Build and Run
1: Enter the project directory

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./waist-control 3

### Input(s)
The input for waist-control (first argument) is an identitifier for the output
file name.

### Commands
A joystick is used for the control of the waist.

    dpad up    : move waist forwards
    dpad down  : move waist backwards
    button 6 & : record data when waist is moving
        (dpad up | dpad down)

### Output(s)
The script outputs multiple files with different formats.
- `qWaistOut` : (qWaist, qTorso, qLArm0, ..., qLArm6, qRArm0, ..., qRArm6)
- `dqWaistOut` : (dqWaist)
- `currSomWaistOut` : (currSomWaist)
- `currCmdWaistOut` : (currCmdWaist)
- `timeWaistOut` : (time)

## Controls
TODO

The joystick buttons 1-4 are used for issuing commands, with buttons 5-8 and the right axis to determine which body part to move. Buttons 5 and 6 are for left/right shoulders, 7 and 8 for left/right arms, left joystick down for torso, and left directional pad (up/down) for waist.

While moving, any time the buttons are released, the joints will stop moving and lock up whether or not they have reached their targets. If the same direction is applied after the joint stops moving, it will start moving again and reach for the previous target (and not move the target one further step down). If the opposite direction is pressed then the target will be moved one step size back, with target set to the starting pose.

In general, button 1 toggles the pose list down one (for arms) or move the join counterclock wise for 1 step size. For example, with the current setting pressing and holding buttons 5 and 1 would move the left shoulder up 0.1 radian. Holding buttons 1 and 8, and the arm will move to the next configuration from the poses.txt file. Button 3 goes in the opposite direction of button 1. Button 2 sets the target of the body part to its default (all zeros). Button 4 sets the current position of the join to its target, can be used for fine-tuning poses.

The left directional pad is used for waist, going up to increase the waist angle and down to decrease it. No position target or default/presets exist for the waist.
