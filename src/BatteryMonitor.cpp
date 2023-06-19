#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor() {
    battery_level = -1.0;
}

void BatteryMonitor::begin() {
    pinMode(BATTERY_PIN, INPUT); 
}

// update the battery percentage: https://www.pangodream.es/esp32-getting-battery-charging-level
void BatteryMonitor::updateBatteryLevel() {
    float pin_ratio = 3300.0 / 4096.0;  // max val of ADC pin input divided by max value that pin will read in
    float input_ratio = VIN / VOUT;     // actual max battery voltage divided by the actual voltage coming out of the "voltage divider"
    battery_level = ((pin_ratio * input_ratio * (float) analogRead(BATTERY_PIN) * (1.0/1000.0)) / VIN) * 100.0;
}

float BatteryMonitor::getBatteryLevel() {
    return battery_level;
}