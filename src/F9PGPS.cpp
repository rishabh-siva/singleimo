#include "F9PGPS.h"

F9PGPS::F9PGPS() {}

void F9PGPS::begin() {
    Wire.begin();
    if (this->myGNSS.begin() == false)
    {
        Serial.println("GNSS not detected at default I2C address. Please check wiring. Freezing.");
        while(1);
    }
    this->myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);      // Save (only) the communications port settings to flash and BBR
    this->myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);   // Make sure the library is only passing GGA messages
}

void F9PGPS::print() {
    float latitude = this->myGNSS.getLatitude();
    float longitude = this->myGNSS.getLongitude();
    float altitude = this->myGNSS.getAltitude();
    float heading = this->myGNSS.getHeading();
    float speed = this->myGNSS.getGroundSpeed();
    Serial.print("Latitude: ");
    Serial.println(latitude / 10000000., 7);
    Serial.print("Longitude: ");
    Serial.println(longitude / 10000000., 7);
    Serial.print("Altitude: ");
    Serial.println(altitude, 6);
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("Speed: ");
    Serial.println(speed);
    Serial.print("Fix Type: ");
    Serial.println(this->myGNSS.getFixType());
}

bool F9PGPS::isReady() { return this->myGNSS.getFixType() >= 2; }  // check if there is fix

void F9PGPS::setLatOffset(double lat_offset) { this->lat_offset = lat_offset; }
void F9PGPS::setLonOffset(double lon_offset) { this->lon_offset = lon_offset; }

// Voidwalker is ready for AI
bool F9PGPS::aiReady() { return isReady() && this->is_gnss_accurate; } 


