#include "MotorController.h"

MotorController::MotorController() {}

void MotorController::begin(int left_speed, int right_speed) {
    // power and ground
    pinMode(BOTTOM_LEFT_GROUND, OUTPUT);
    pinMode(BOTTOM_LEFT_POWER, OUTPUT);
    pinMode(BOTTOM_RIGHT_POWER, OUTPUT);
    pinMode(BOTTOM_RIGHT_GROUND, OUTPUT);
    pinMode(TOP_LEFT_GROUND, OUTPUT);
    pinMode(TOP_LEFT_POWER, OUTPUT);
    pinMode(TOP_RIGHT_POWER, OUTPUT);
    pinMode(TOP_RIGHT_GROUND, OUTPUT);

    // PWM for speed
    pinMode(BOTTOM_LEFT_ENABLE, OUTPUT);
    analogWrite(BOTTOM_LEFT_ENABLE, left_speed);
    pinMode(BOTTOM_RIGHT_ENABLE, OUTPUT);
    analogWrite(BOTTOM_RIGHT_ENABLE, right_speed);
    pinMode(TOP_LEFT_ENABLE, OUTPUT);
    analogWrite(TOP_LEFT_ENABLE, left_speed);
    pinMode(TOP_RIGHT_ENABLE, OUTPUT);
    analogWrite(TOP_RIGHT_ENABLE, right_speed);
}

// all digitalWrites should go through this function because it's used in asynDelay as well as controls the turn off battery reminder
void MotorController::drive(String command) {
    if(! mutex) {
        // write voltage commands to motors
        if(command == "forward") {
            digitalWrite(BOTTOM_LEFT_POWER, HIGH); 
            digitalWrite(BOTTOM_RIGHT_POWER, HIGH);
            digitalWrite(TOP_LEFT_POWER, HIGH); 
            digitalWrite(TOP_RIGHT_POWER, HIGH);
            digitalWrite(BOTTOM_LEFT_GROUND, LOW);
            digitalWrite(BOTTOM_RIGHT_GROUND, LOW); 
            digitalWrite(TOP_LEFT_GROUND, LOW);
            digitalWrite(TOP_RIGHT_GROUND, LOW); 
        }
        else if(command == "forward_left") {
            digitalWrite(BOTTOM_RIGHT_POWER, HIGH);
            digitalWrite(BOTTOM_RIGHT_GROUND, LOW); 
            digitalWrite(TOP_RIGHT_POWER, HIGH);
            digitalWrite(TOP_RIGHT_GROUND, LOW); 
            digitalWrite(BOTTOM_LEFT_POWER, LOW); 
            digitalWrite(BOTTOM_LEFT_GROUND, LOW);
            digitalWrite(TOP_LEFT_POWER, LOW); 
            digitalWrite(TOP_LEFT_GROUND, LOW);
        }
        else if(command == "forward_right") {
            digitalWrite(BOTTOM_RIGHT_POWER, LOW);
            digitalWrite(BOTTOM_RIGHT_GROUND, LOW); 
            digitalWrite(TOP_RIGHT_POWER, LOW);
            digitalWrite(TOP_RIGHT_GROUND, LOW); 
            digitalWrite(BOTTOM_LEFT_POWER, HIGH); 
            digitalWrite(BOTTOM_LEFT_GROUND, LOW);
            digitalWrite(TOP_LEFT_POWER, HIGH); 
            digitalWrite(TOP_LEFT_GROUND, LOW); 
        }
        else if(command == "backward") {
            digitalWrite(BOTTOM_LEFT_POWER, LOW); 
            digitalWrite(BOTTOM_RIGHT_POWER, LOW);
            digitalWrite(TOP_LEFT_POWER, LOW); 
            digitalWrite(TOP_RIGHT_POWER, LOW);
            digitalWrite(BOTTOM_LEFT_GROUND, HIGH);
            digitalWrite(BOTTOM_RIGHT_GROUND, HIGH); 
            digitalWrite(TOP_LEFT_GROUND, HIGH);
            digitalWrite(TOP_RIGHT_GROUND, HIGH); 
        }
        else if(command == "backward_left") {
            digitalWrite(BOTTOM_RIGHT_POWER, LOW);
            digitalWrite(BOTTOM_RIGHT_GROUND, HIGH); 
            digitalWrite(TOP_RIGHT_POWER, LOW);
            digitalWrite(TOP_RIGHT_GROUND, HIGH); 
            digitalWrite(BOTTOM_LEFT_POWER, LOW); 
            digitalWrite(BOTTOM_LEFT_GROUND, LOW);
            digitalWrite(TOP_LEFT_POWER, LOW); 
            digitalWrite(TOP_LEFT_GROUND, LOW);
        }
        else if(command == "backward_right") {
            digitalWrite(BOTTOM_RIGHT_POWER, LOW);
            digitalWrite(BOTTOM_RIGHT_GROUND, LOW); 
            digitalWrite(TOP_RIGHT_POWER, LOW);
            digitalWrite(TOP_RIGHT_GROUND, LOW); 
            digitalWrite(BOTTOM_LEFT_POWER, LOW); 
            digitalWrite(BOTTOM_LEFT_GROUND, HIGH);
            digitalWrite(TOP_LEFT_POWER, LOW); 
            digitalWrite(TOP_LEFT_GROUND, HIGH);
        }
        else if(command == "left") {
            digitalWrite(BOTTOM_RIGHT_POWER, HIGH);
            digitalWrite(BOTTOM_RIGHT_GROUND, LOW); 
            digitalWrite(TOP_RIGHT_POWER, HIGH);
            digitalWrite(TOP_RIGHT_GROUND, LOW); 
            digitalWrite(BOTTOM_LEFT_POWER, LOW); 
            digitalWrite(BOTTOM_LEFT_GROUND, HIGH);
            digitalWrite(TOP_LEFT_POWER, LOW); 
            digitalWrite(TOP_LEFT_GROUND, HIGH);
        }
        else if(command == "right") {
            digitalWrite(BOTTOM_RIGHT_POWER, LOW);
            digitalWrite(BOTTOM_RIGHT_GROUND, HIGH); 
            digitalWrite(TOP_RIGHT_POWER, LOW);
            digitalWrite(TOP_RIGHT_GROUND, HIGH); 
            digitalWrite(BOTTOM_LEFT_POWER, HIGH); 
            digitalWrite(BOTTOM_LEFT_GROUND, LOW);
            digitalWrite(TOP_LEFT_POWER, HIGH); 
            digitalWrite(TOP_LEFT_GROUND, LOW);
        }

        // updates the batter reminder
        battery_timer_start = millis();
    }
}

// shouldn't be mutexed so can be used as an e-stop
// can't use asyncDelay because has caused an infinite loop
void MotorController::stop() {
    digitalWrite(BOTTOM_LEFT_GROUND, LOW);
    digitalWrite(BOTTOM_LEFT_POWER, LOW); 
    digitalWrite(BOTTOM_RIGHT_POWER, LOW);
    digitalWrite(BOTTOM_RIGHT_GROUND, LOW); 
    digitalWrite(TOP_LEFT_GROUND, LOW);
    digitalWrite(TOP_LEFT_POWER, LOW); 
    digitalWrite(TOP_RIGHT_POWER, LOW);
    digitalWrite(TOP_RIGHT_GROUND, LOW); 
}

void MotorController::setLeftSpeed(int speed) {
    analogWrite(BOTTOM_LEFT_ENABLE, speed);
    analogWrite(TOP_LEFT_ENABLE, speed);
}

void MotorController::setRightSpeed(int speed) {
    analogWrite(BOTTOM_RIGHT_ENABLE, speed);
    analogWrite(TOP_RIGHT_ENABLE, speed);
}
