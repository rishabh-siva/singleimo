#include "Arduino.h"

class BatteryMonitor {
    const int BATTERY_PIN = 36;
    const float VIN = 12.5;
    const float VOUT = 3.023;
    float battery_level;

    public:
        const float BATTERY_THRESH = 50.0;
        BatteryMonitor();

        void begin();
        void updateBatteryLevel();
        float getBatteryLevel();
};