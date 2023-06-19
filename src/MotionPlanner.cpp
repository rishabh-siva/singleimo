#include "MotionPlanner.h"

MotionPlanner::MotionPlanner() {}

void MotionPlanner::begin() {
    memory.begin("motion", false);   // 15 char name limit; false = read/write

	// TODO: replaced by nav2
	// motor_controller.begin(memory.getInt("left_speed", 127), memory.getInt("right_speed", 127));
}

void MotionPlanner::joystick(int x, int y) {
	if(mode == "manual") {
		// print x and y
		Serial.print("x ");
		Serial.print(x);
		Serial.print(", y ");
		Serial.println(y);

		// drive
		int CENTER_MIN = 70; int CENTER_MAX = 170;
		int DRIVE_MIN = 25; int DRIVE_MAX = 195;
		if(y > CENTER_MAX) {
			// TODO: replaced by nav2
			// if(x > DRIVE_MAX) 		motor_controller.drive("forward_right", 0);
			// else if(x < DRIVE_MIN) 	motor_controller.drive("forward_left", 0);
			// else 					motor_controller.drive("forward", 0);
		}
		else if(y < CENTER_MIN) {
			// TODO: replaced by nav2
			// if(x > DRIVE_MAX)		motor_controller.drive("backward_right", 0);
			// else if(x < DRIVE_MIN)	motor_controller.drive("backward_left", 0);
			// else					motor_controller.drive("backward", 0);
		}
		else {
			// TODO: replaced by nav2
			// if(x > DRIVE_MAX)		motor_controller.drive("right", 0);
			// else if(x < DRIVE_MIN)	motor_controller.drive("left", 0);
			// else 					motor_controller.stop();
		}
	}
}

// TODO: replaced by nav2
// void MotionPlanner::run() {
// 	// sync async delay timer for user commands
// 	motor_controller.asyncDelay(0);

// 	// avoid obstacles if needed
// 	if(mode != "") {
//         bool is_left_touched = !digitalRead(TOUCH_LEFT);
//         bool is_right_touched = !digitalRead(TOUCH_RIGHT);
// 		if(is_left_touched || is_right_touched) {
// 			motor_controller.drive("backward", 1000);
// 			if(is_left_touched && is_right_touched) turn(90, true);
// 			else if(is_left_touched && !is_right_touched) turn(45, true);
// 			else if(!is_left_touched && is_right_touched) turn(-45, true);
// 		}
// 	}
// }

///////////////// Getters and Setters ///////////////////////////////
String MotionPlanner::getMode() { return this->mode; }
int MotionPlanner::getModeInt() {
	if(mode == "manual") return 1;
	if(mode == "gps") return 2;
	else return 0;
}
void MotionPlanner::setMode(String mode) { 
	this->mode = mode; 

	// TODO: replaced by nav2
	// if(mode == "") motor_controller.stop();
	// if(mode == "gps") motor_controller.drive("forward", 4000); // need to initializing drive straight after user changes mode, otherwise the queued turn will be random
}

void MotionPlanner::setLeftSpeed(int speed) { 
	memory.putInt("left_speed", speed);

	// TODO: replaced by nav2
	// motor_controller.setLeftSpeed(speed); 
}
void MotionPlanner::setRightSpeed(int speed) { 
	memory.putInt("right_speed", speed);

	// TODO: replaced by nav2
	// motor_controller.setRightSpeed(speed); 
}

// TODO: replaced by nav2
// int MotionPlanner::minutesSinceLastCommand() { return motor_controller.minutesSinceLastCommand(); }