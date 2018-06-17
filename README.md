# offline-com
This is the repo for Krang pose control files to be used in offline center of mass testing. The main file (exe/pose-control.cpp) reads in a series of arm poses (6 joints not including shoulder) toggleable via the joystick and uses fix step-sies for moving torso/shoulders. Waist is moved based on velocity and without fixed position targets. 

# Controls 
The joystick buttons 1-4 are used for issuing commands, with buttons 5-8 and the right axis to determine which body part to move. Buttons 5 and 6 are for left/right shoulders, 7 and 8 for left/right arms, left joystick down for torso, and left directional pad (up/down) for waist. 

While moving, any time the buttons are released, the joints will stop moving and lock up whether or not they have reached their targets. If the same direction is applied after the joint stops moving, it will start moving again and reach for the previous target (and not move the target one further step down). If the opposite direction is pressed then the target will be moved one step size back, with target set to the starting pose.  

In general, button 1 toggles the pose list down one (for arms) or move the join counterclock wise for 1 step size. For example, with the current setting pressing and holding buttons 5 and 1 would move the left shoulder up 0.1 radian. Holding buttons 1 and 8, and the arm will move to the next configuration from the poses.txt file. Button 3 goes in the opposite direction of button 1. Button 2 sets the target of the body part to its default (all zeros). Button 4 sets the current position of the join to its target, can be used for fine-tuning poses. 

The left directional pad is used for waist, going up to increase the waist angle and down to decrease it. No position target or default/presets exist for the waist. 

Button 10 is used to record pose data, which will stored them in a file and display them on screen. 

# IO Files
The arm poses are to be read from poses.txt, one line per arm pose (for both left and right arm, the right arm poses will be multiplied by -1). The pose output readings will be saved in data.txt, 1 line per pose (left arm, right arm, torso, waist, imu reading).
