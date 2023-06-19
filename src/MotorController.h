#include "Arduino.h"

// Controls motors from signals
// Always stops after every command (drive, queuedDrive)
class MotorController {
	// ground pins
	int BOTTOM_LEFT_GROUND = 26;	// bottom left wheel; ground pin, closest to ENABLE
	int BOTTOM_RIGHT_GROUND = 14; 	// bottom right wheel; ground pin, farther from ENABLE
	int TOP_LEFT_GROUND = 23;
	int TOP_RIGHT_GROUND = 19;

	// power pins
	int BOTTOM_LEFT_POWER = 27; 
	int BOTTOM_RIGHT_POWER = 12;
	int TOP_LEFT_POWER = 5; 
	int TOP_RIGHT_POWER = 18;
	
	// enable pins; seems to work better with TOUCH pins, but not sure
	int BOTTOM_LEFT_ENABLE = 22;
	int BOTTOM_RIGHT_ENABLE = 13;
	int TOP_LEFT_ENABLE = 2;
	int TOP_RIGHT_ENABLE = 15;

	// async delay
	unsigned long async_timer_start = 0; // TODO millis overflows after 50 days
	bool mutex = false;
	String queued_command = "";
	int current_ms = 0;

	// battery reminder
	long battery_timer_start = millis();

public:
	// Constructors
	MotorController();
	void begin(int left_speed, int right_speed);	// init pins

	// Functions
	void drive(String command); 					// drive motors; mutexed so future calls won't activate unless the current command finished; these future commands can be stored using queuedDrive
	void stop();
	void setLeftSpeed(int speed);
	void setRightSpeed(int speed);
	//void asyncDelay(int ms); 						// function that stops this class from doing anything else besides its current task, because blocking delays prevent other functions from running, especially syncing with Blynk Cloud
};
