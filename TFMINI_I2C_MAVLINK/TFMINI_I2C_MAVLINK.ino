/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   Tested with Ardupilot by Patrick Poirier December 2017.
   Might containe open licenced code from other sources, thank you ;-)
   More on my github https://github.com/patrickpoirier51
 */



// Arduino MAVLink  http://forum.arduino.cc/index.php?topic=382592.0
// https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/code-overview-object-avoidance.rst

/*
 *  The system id of the message should match the system id of the vehicle 
 *  (default is "1" but can be changed using the SYSID_THISMAV parameter). 
 *  The component id can be anything but MAV_COMP_ID_PATHPLANNER (195) 
 *  or MAV_COMP_ID_PERIPHERAL (158) are probably good choices.
 *  
 * # Define function to send distance_message mavlink message for mavlink based rangefinder, must be >10hz
# http://mavlink.org/messages/common#DISTANCE_SENSOR
def send_distance_message(dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = 0 MAV_DISTANCE_SENSOR_LASER Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    if args.verbose:
        log.debug("Sending mavlink distance_message:" +str(dist))
        */
   
#include "mavlink.h"            // Mavlink interface
#include "mavlink_msg_distance_sensor.h"
#include <Wire.h>
#define bRate 115200            // Baudrate
uint16_t distance = 0;          //Diatance reading
uint8_t wait = 6;              //Delay between I2C readings
uint8_t scale = 10;             //Scale divider  ex. mm to cm

void setup(){ 
 Serial.begin(bRate);
 Wire.begin();
}


void command_heartbeat() {
  int sysid = 100;                            //< ID 1 for this system               
  int compid = MAV_COMP_ID_PATHPLANNER;       //< The component sending the message.
  uint8_t system_type =MAV_TYPE_GCS;         // Define the system type, in this case ground control station
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  //Serial.write(buf, len);
}


void command_distance(uint8_t orient ,uint16_t rngDist) {
  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 30; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 900; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = rngDist; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 1; /*< Onboard ID of the sensor*/
  uint8_t orientation = orient; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class
  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  Serial.write(buf, len);
 // Serial.print (orientation);
  //Serial.print (" - ");
  //Serial.println (current_distance);
}


void wireRead(int Address,  int Data) {
  uint8_t byteLow;
  uint8_t byteHigh;
  Wire.requestFrom(Address, Data);    // Adress - Bytes (only one with Attiny)
  while (Wire.available()) { // slave may send less than requested
    byteLow = Wire.read();    // receive a byte as character
   //Serial.print(Wire.read(), HEX);         // print the character
   delay(wait);
   }
     Wire.requestFrom(Address, Data);    // Adress - Bytes (only one with Attiny)
  while (Wire.available()) { // slave may send less than requested
   byteHigh = Wire.read();    // receive a byte as character
   //Serial.println(Wire.read(), HEX);         // print the character
   delay(wait);
   }   
  distance= ((uint16_t)((byteLow) + (byteHigh*256))/scale);
}


void loop() {
/*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
//command_heartbeat();

  wireRead(3, 1);
  //Serial.print (distance);
  //Serial.print (" - ");
 command_distance(24 ,distance);

  wireRead(4, 1);
  //Serial.print (distance);
  //Serial.print (" - ");
 command_distance(7 ,distance);
 
  wireRead(5, 1);
  //Serial.print (distance);
  //Serial.print (" - ");
  command_distance(0 ,distance);
 
  wireRead(6, 1);
  //Serial.println (distance);
  command_distance(1 ,distance);
  //delay(wait);
  
}



