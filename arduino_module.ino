
/*
  Software serial multple serial test
 
 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.
 
 The circuit: 
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 
 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts, 
 so only the following can be used for RX: 
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 
 Not all pins on the Leonardo support change interrupts, 
 so only the following can be used for RX: 
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 
 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example
 
 This example code is in the public domain.
 
 */
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
SoftwareSerial mySerial(11, 10); // RX, TX


Adafruit_BMP280 bme; // I2C
void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  
  if (!bme.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    //while (1);
  }   
}


float pres;
float old_pres;
float d_pres;
float smoothed_pres;
float smoothed_d_pres;
unsigned long msec,old_msec;
float FALLING_LEVEL = -1.0f;
float RISING_LEVEL = 1.0f;
float dt;
float old_sec;
String PILC = "&PILC,PDA1,";

  
void loop() // run over and over
{

  if (mySerial.available())
    Serial.write(mySerial.read());
  if (Serial.available())
    mySerial.write(Serial.read());
  
  msec = millis();
  dt = msec - old_msec;
  if(dt >= 100){
    pres = bme.readAltitude(1013.25);
    smoothed_pres = 0.97*smoothed_pres + 0.3*pres;
  
    d_pres = (pres - old_pres)*1000/dt;
    old_sec = msec;
    old_pres = pres;
  
    smoothed_d_pres = 0.9*smoothed_d_pres + 0.1*d_pres;
    PILC += ((int)pres);
    PILC += ",";
    int int_part =((int)smoothed_d_pres);
    int dec_part = ((int)smoothed_d_pres*10)-int_part * 10;
    PILC += int_part;
    PILC += ".";
    PILC += dec_part;
    PILC += "*69";
  }
// MAKE BEEP
// Variables are : Period of beep, DutyCycle of beep, Frequency of sound
// When falling, period is infitine, dutycycle 100% and freq get lower with d_pres
if(smoothed_d_pres < FALLING_LEVEL){
	
} 
}

