// touch defines
#define TOUCH_LEFT 32
#define TOUCH_RIGHT 33

// includes
#include "Arduino.h"
#include <Preferences.h> // https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
#include "Constants.h"

class MotionPlanner {
	String mode = "";					// user modes
	Preferences memory;					// storing variables on disk

public:
	// Constructors
	MotionPlanner();
	void begin();

	// Functions
	void joystick(int x, int y); // Blynk joystick
	void run();

	// Getters and Setters
	String getMode();
	int getModeInt();
	void setMode(String mode);
	void setLeftSpeed(int speed);
	void setRightSpeed(int speed);
	int minutesSinceLastCommand();
};
