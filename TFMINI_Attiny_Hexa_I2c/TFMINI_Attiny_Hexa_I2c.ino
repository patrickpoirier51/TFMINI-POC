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
   This is a MaxBotix MaxsonarI2CXL emulator program that can be used with a
   Benewake TFMINI TOF Laser rangefinder. This allows the serial stream to be converted
   to an I2C message that is more convenient for most Flight Controler that are lacking 
   UARTS. 
   Tested with Ardupilot by Patrick Poirier December 2017.
   Might containe open licenced code from other sources, thank you ;-)
   More on my github https://github.com/patrickpoirier51
 */


#include "TinyWireS.h"    
#define I2C_SLAVE_ADDR  0x3 

#include "SoftwareSerial.h"
const int Rx = 3; // this is physical pin 2 - you do not need to connect this to DFPlayer
const int Tx = 4; // this is physical pin 3 - connect to RX pin on DFPlayer
SoftwareSerial mySerial(Rx, Tx);

volatile byte SendCount;
volatile uint8_t sendByte;
uint8_t Framereceived[9];
uint8_t index;
uint8_t distLow;
uint8_t distHigh;

/*uint16_t distance;
uint16_t offset = 0;
uint16_t scale = 1;
uint16_t strength;
uint16_t strLimit = 1175;
uint16_t distMin = 465; 
*/

uint8_t Checksum(uint8_t *data, uint8_t length)
 {
     uint16_t  count;
     uint16_t  Sum = 0;
     
     for (count = 0; count < length; count++)
         Sum = Sum + data[count];
     return (Sum); 
 }

 

void setup() {
pinMode(Rx, INPUT);
pinMode(Tx, OUTPUT);
mySerial.begin(9600);
TinyWireS.begin(I2C_SLAVE_ADDR); // join i2c bus with address #8
TinyWireS.onRequest(requestEvent); // register event
SendCount = 0;
//delay(100);
}

/*
Standard output= 
Byte1-2   Byte3   Byte4   Byte5     Byte6     Byte7     Byte8    Byte9
0x59 59   Dist_L  Dist_H  Strength_L  Strength_H   Reserved   Raw.Qual  CheckSum_
*/

void readlaser(){     
  if (mySerial.available() > 0) {
      uint8_t inChar = mySerial.read();
       if((inChar=='Y')&& (index==0)){
        Framereceived[index]=inChar;
        index++;
       }
       else{
         if( Framereceived[0]=='Y'){
          if(index<8){
            Framereceived[index]=inChar;
            index++;
          }
          else{
             Framereceived[index]=inChar;
             if( Framereceived[1]=='Y'){
              if(Checksum(Framereceived, 8)==Framereceived[8]){
              //distance= (uint16_t)(Framereceived[2] + (Framereceived[3]*256));
              distLow = (uint8_t)(Framereceived[2]); 
              distHigh = (uint8_t)(Framereceived[3]);
              //strength = (uint16_t)(Framereceived[4] + (Framereceived[5]*256));

/*
                if ( (strength > strLimit) && (distance <= distMin ))  {
                  distance = distance - ((abs(strength - strLimit))/3) ;
                    }  
                  if ((strength > strLimit) && (distance > distMin)){
                    distance = 140;
                      } 
                      
                    distance = constrain(distance, 140, 9000);                    
                    distance = ((distance - offset)/scale); 
 */
                   // mySerial.println(distance);
                   }   
               }           
             for(uint8_t i=0;i<8;i++){
             Framereceived[i]=0;
             }
            index=0;
          }
       }
    }
  }

}

/*
void requestEvent() {
TinyWireS.send(0x56); // respond with message of 6 bytes
  // as expected by master
}
*/

void requestEvent()
{  
  SendCount++; //increment this
  switch (SendCount)
  {
    case 1:
      sendByte = distLow;
      break;
    case 2:
      sendByte = distHigh;
     /* break;
    case 3:
      sendByte = 0x0D;  //Carriage Return*/
      SendCount=0;
      break;
  }

  TinyWireS.send(sendByte);
}

void loop() {
readlaser();
TinyWireS_stop_check();
}

