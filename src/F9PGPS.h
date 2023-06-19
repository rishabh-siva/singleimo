#include <Arduino.h>
#include <Wire.h>                                   // Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>   // The library used for the new GNSS

class F9PGPS {
    double lat_offset = 0.0;	// constant error correction for lat/lon
	double lon_offset = 0.0;
    
    public:
        bool is_gnss_accurate = false; 
        SFE_UBLOX_GNSS myGNSS;  // Declare an object for the GNSS

        F9PGPS();           // Constructor
        
        void begin();       // init GNSS
        void print();       // Print data
        bool isReady();
        void setLatOffset(double lat_offset);
	    void setLonOffset(double lon_offset);
        bool aiReady();
};