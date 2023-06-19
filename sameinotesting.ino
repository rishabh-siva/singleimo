//VOIDWALKER 2 INCLUDES UPTO SETUP
TaskHandle_t micro_ros_task; 
// Blynk defines
#define BLYNK_TEMPLATE_ID       "TMPLkUH7AMjE"
#define BLYNK_TEMPLATE_NAME     "Voidwalker"
#define BLYNK_FIRMWARE_VERSION  "0.1.0"
#define BLYNK_PRINT             Serial
#define APP_DEBUG

// external includes
#include "Blynk_ESP32/BlynkEdgent.h"
#include <Wire.h>

// project includes
#include "src/F9PGPS.h"
#include "src/MotionPlanner.h"
#include "src/BatteryMonitor.h"

// GPS
F9PGPS gnss;
BatteryMonitor battery;           // Battery 
MotionPlanner motion_planner;     // Motion Planner

// Waypoints
double goal_lat = 0.0;            // Goal Lat
double goal_lon = 0.0;            // Goal Lon
int waypoint_update_number = 0;
int waypoint_type = 1;            // 1 = autonomous; 2 = manual

// Timers Setup
unsigned long blynk1s_timer = 1000; // Blynk 1s interval in milliseconds
unsigned long blynk1s_PreviousMillis = 0; // Previous time value for Blynk 1s
unsigned long blynk5s_timer = 5000; // Blynk 5s interval in milliseconds
unsigned long blynk5s_PreviousMillis = 0; // Previous time value for Blynk 5s

// To solve timing issues between GNSS detection and Blynk
unsigned long gnssDetectionInterval = 200;  // Interval for checking GNSS device (in milliseconds)
unsigned long previousGnssDetectionMillis = 0;

// Blynk
WidgetMap blynk_map(V2);          // Map; uses decimal degrees; only works with double or String, not float

//VOIDWALKER ROS 2 INCLUDES UPTO ERROR LOOP FUNCTION

#include <micro_ros_arduino.h>

#include <string>
#include <rosidl_runtime_c/string.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <example_interfaces/msg/string.h>

#include "src/MotorController.h"

rcl_subscription_t subscriber;
rcl_publisher_t command_publisher;
geometry_msgs__msg__Twist msg;
example_interfaces__msg__String ros_command;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
MotorController mc;


#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}


//VOIDWALKER 2 ALL USER DEFINED FUNCTIONS AND OTHER STUFF 

// TODO: replaced by nav2
// // drive robot to next waypoint 
// void updatePath() {
//     path_planner.updateGPSPlan(gps.getLatitude(), gps.getLongitude());          // calculates the direction and distance to the next waypoint    
  
//     if(battery_level > BATTERY_THRESH && BlynkState::get() == MODE_RUNNING) {   // if battery is high enough
//         motion_planner.run();                                                   // backs up if touching obstacle; sync with motor controller

//         if( aiReady() && motion_planner.getMode() == "gps" ) {
//             motion_planner.turn(path_planner.pollTurnAngle(), true);            // turn if the heading is calibrated
//             motion_planner.aiDrive(path_planner.getDistToGoal());               // drive forwards towards the goal
//         }
//     }
//     else motion_planner.setMode("");
// }

// update error messages
String updateErrorMessages() { 
    String message = "";
    String color = "";
    
    if(0.0 < battery.getBatteryLevel() && battery.getBatteryLevel() <= battery.BATTERY_THRESH) {  // skipping 0 because then the message will display when the user unplugs the battery
        message = "Low battery; stopping motors."; 
        color = Constants::RED;
    }
    else if(! gnss.isReady()) {
        message = "Waiting for GPS fix.";
        color = Constants::YELLOW;
    }
    else if(! gnss.is_gnss_accurate) {
        message = "Check the GPS accurate button.";               // needs to come after checking gps.isReady()
        color = Constants::YELLOW;
    }
    else {
        message = "No errors";
        color = Constants::GREEN;
    }
    
    Blynk.setProperty(V24, "color", color);
    return message;
}

////////////////// Blynk I/O //////////////////////////////////////////////////////////////////////////////////////////////////////

// read in Blynk data
BLYNK_WRITE(V0) { motion_planner.joystick(param[0].asInt(), param[1].asInt()); }  // joystick
BLYNK_WRITE(V5) { goal_lat = param.asDouble(); }                                  // goal latitude
BLYNK_WRITE(V6) { goal_lon = param.asDouble(); }                                  // goal longitude
BLYNK_WRITE(V10) {                                                                // which motion planner should be active
    switch(param.asInt()) {
        case 0: motion_planner.setMode(""); break;
        case 1: motion_planner.setMode("manual"); break;
        case 2: motion_planner.setMode("gps"); break;
    }
}
     
BLYNK_WRITE(V7) { gnss.setLatOffset(param.asDouble()); }                           // constant error correction for lat/lon
BLYNK_WRITE(V14) { gnss.setLonOffset(param.asDouble()); } 
BLYNK_WRITE(V17) { motion_planner.setLeftSpeed(param.asInt()); }                  // updates motor speed
BLYNK_WRITE(V19) { motion_planner.setRightSpeed(param.asInt()); }
BLYNK_WRITE(V20) { ESP.restart(); }
BLYNK_WRITE(V25) { gnss.is_gnss_accurate = param.asInt(); }
BLYNK_WRITE(V27) { waypoint_update_number = param.asInt(); }

// TODO: replaced by nav2
// BLYNK_WRITE(V28) { 
//     Serial.println("command: " + String(param.asInt()));
//     path_planner.saveWaypoints(param.asInt(), waypoint_update_number, goal_lat, goal_lon, waypoint_type); 
// }
BLYNK_WRITE(V30) { waypoint_type = param.asInt(); }

// write to Blynk app; can only accept 10 items per second
void updateBlynk1s() {
    Blynk.virtualWrite(V3, gnss.myGNSS.getHeading());                      // compass heading
    Blynk.virtualWrite(V15, !digitalRead(TOUCH_LEFT));                      // state of touch sensors
    Blynk.virtualWrite(V16, !digitalRead(TOUCH_RIGHT));
}
void updateBlynk5s() {
    // display values
    Blynk.virtualWrite(V1, gnss.aiReady());                                 // if GPS is tracking satellites or not
    Blynk.virtualWrite(V18, WiFi.RSSI());                                   // WiFi signal strength
    Blynk.virtualWrite(V22, motion_planner.getModeInt());                   // mode; this is separate from setting the mode in Blynk because oftentimes it says one state but really is another
    battery.updateBatteryLevel();
    Blynk.virtualWrite(V23, battery.getBatteryLevel());                                 // battery level
    Blynk.virtualWrite(V24, updateErrorMessages());                         // prints error messages for user to see why AI mode not working

    // TODO: replaced by nav2
    // display waypoints info
    // vector<Waypoint> waypoints = path_planner.getWaypoints();
    // for(int i = 1; i < waypoints.size(); i++) {                             // i=0 is empty, because BlynkMap is using 0 for the robot
    //     String type = (waypoints[i].waypoint_type == 2) ? "M" : "";
    //     blynk_map.location(i, waypoints[i].latitude, waypoints[i].longitude, String(i) + type); // waypoint on map; works with doubles not floats
    // }
    // Blynk.virtualWrite(V29, path_planner.getWaypointInfo());                // displays current waypoints over the total
} 


// VOIDWALKER ROS 2 ALL USER DEFINED FUNCTIONS

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

String getDriveCommand(double x,double z){
  String command;
  if ( z == 0.0 ){  
    command = (x > 0.0) ? "forward" : "backward";
  }
  else if (x == 0.0){
    command = (z > 0.0) ? "left" : "right";
  }
  else{
    if (x > 0.0){
      command = (z > 0.0) ? "forward_left" : "forward_right";
    }
    else if (x < 0.0){
      command = (z > 0.0) ? "backward_left" : "backward_right";
    }
  }
  return command;
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // get and set the command
  String command = getDriveCommand(msg->linear.x, msg->angular.z);
  mc.drive(command);
  if(msg->linear.x == 0 && msg->angular.z == 0) mc.stop();

  rosidl_runtime_c__String ros_string;
  ros_string.data = (char*)command.c_str();
  ros_string.size = command.length();
  
  ros_command.data = ros_string;

  RCSOFTCHECK(rcl_publish(&command_publisher,&ros_command, NULL))
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
}

void setup()
{

//SETUP OF VOIDWALKER 2 

    // setup software
    Serial.begin(115200);                 // Serial                
    gnss.begin();                         // Starting GNSS
    BlynkEdgent.begin();                  // Blynk Cloud

    // setup hardware
    pinMode(TOUCH_LEFT, INPUT_PULLUP);    // touch; these always need to be pullup otherwise non-pushed state is undefined because there is no connection between power and ground
    pinMode(TOUCH_RIGHT, INPUT_PULLUP);   // touch
    motion_planner.begin();               // calls setup for Motor Controller pins
    battery.begin();                      // setup battery

    // reset app
    Blynk.virtualWrite(V1, false);        // turn off GPS Ready LED

    // wait for Blynk
    while(BlynkState::get() != MODE_RUNNING) {
        BlynkEdgent.run();
    }
    
//SETUP OF VOIDWALKER ROS 2

  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create command publisher
  RCCHECK(rclc_publisher_init_default(
    &command_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(example_interfaces,msg,String),
    "cmd_command"
  ))

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Motor Controller setup 
  mc.begin(250,250); // initialized left and right speed to 0
  
  xTaskCreatePinnedToCore(micro_ros_fn,"micro_ros_task",10000,NULL,1,&micro_ros_task,0);                         
  delay(500); 
         
}

void loop() {

    // Get the current time
    unsigned long currentMillis = millis();

    // Timer 1: Execute every timer1Interval milliseconds
    if (currentMillis - blynk1s_PreviousMillis >= blynk1s_timer) {
        blynk1s_PreviousMillis = currentMillis;
        updateBlynk1s();
    }

    // Timer 2: Execute every timer2Interval milliseconds
    if (currentMillis - blynk5s_PreviousMillis >= blynk5s_timer) {
        blynk5s_PreviousMillis = currentMillis;
        updateBlynk5s();
    }

    // Check GNSS device periodically
    if (currentMillis - previousGnssDetectionMillis >= gnssDetectionInterval) {
        previousGnssDetectionMillis = currentMillis;
        if(gnss.myGNSS.checkUblox() && (gnss.myGNSS.getFixType() >= 2)){
            gnss.print();       // printing GNSS data
            blynk_map.location(0, gnss.myGNSS.getLatitude()/10000000., gnss.myGNSS.getLongitude()/10000000., "robot");
            Blynk.virtualWrite(V4, gnss.myGNSS.getGroundSpeed());                   // speed in knots
        }
    }
    BlynkEdgent.run();
}

void micro_ros_fn( void * parameter )
{
  Serial.print("Micro_ros is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;)
  {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  } 
}





