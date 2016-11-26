
/*
  Program to do long-term temperature data logging of outdoor temperature. 
  From Datalogger_BaseCode_Temperature v3.
  v3 - designed to work through the Grove interface (code assembly with logger components, verify stability and comparative readings of T sensor).
  v4 - minor modifications to eliminate the Grove physical interface.
  
  This program is adapted from the basis for an application for data logging and test control: Datalogger_BaseCode
  Examples:   Temperature logging (one or two temperatures)
              Battery discharge test monitoring voltage vs time, with  shut-off and test complete alarm.
 Datalogger_BaseCode.ino
 Created 30 April 2016
 by Alan Hutchenreuther

Log interval:  5 minutes:
Data record:  Date/ 24-hour Time/ Log duration (seconds)/ Temperature (degF)
Data storage:  2GB microSD
Data capacity:  months or longer.

Hardware configuration for base system (v2 & v3 code):
Arduino Uno R3
Grove modular connector shield.
Grove LCD
ChronoDot RTC board.
Adafruit MicroSD breakout board.

Hardware configuration for target system:
Arduino Uno R3 (Inland clone)
SoarkFun protoshield with MicroSD feature.  (SPI interface to MicroSD)
Standard LCD with I2C interface through Spikenzie I2C port expander 
  (Spikenzie_RGBLCDshield library, adapted from Adafruit_RGBLCDshield but without buttons.))
ChronoDot RTC board. (I2C interface)
Yellow LED to indicate status: log (.5 sec blink) or error (solid ON).
1 DS18B20 temperature sensor( OneWire interface, pin D3)
To supply power, a 9V AC adapter with 20' extension and in-line powewr switch to be provided.

The target system is to replace the original data logger coded in:
TemperatureDisplay_V8_3_DigitalTempTimeLoggingRTC, without a LCD display.   Original hardware uses:
Arduino UNO R3 board with prototype breadboard (from Arduino Starter kit.)
High Temperature DS18B20 temperature sensor,
SparkFun protoshield with MicroSD feature.  (SPI interface to MicroSD)
Yellow LED to indicate status.

Operating Note:  When swapping out microSD cards, remove power.   Failure to do so will
prevent future logging to continue without error indication.

Code changes:
20160607 - Version v4
Replace Grove LCD libraries with Adafruit/Spikenzie libraries.   Instantiate object 'lcd' with new library.
Add "Log" message to LCD display when logging takes place.
20160610 - Version v5
Remove lcd.clear(1) to reduce blinking 
Reduce sig-figs on temperature display.
Set microSD CS pin to 8 instead of 10, for SparkFun shield.
Verify Hardware jumpers present for I2C:  SJ1 & SJ2 jumpers present. microSD_Shield_v14.pdf

*/

// include the library code:
// Reference Arduino Build Process for include path description.
//   https://www.arduino.cc/en/Hacking/BuildProcess    and
//   https://www.arduino.cc/en/hacking/libraries
// See Reference for Libraries.Contributed Libraries for more instructions.
// Putting library in the search path allows short reference using <filename> syntax.
#include <Arduino.h>  // Reference Arduino Build Process.
#include <Wire.h>  // For LCD and ChronoDot RTC Option
#include <Time.h>  // For Time manipulation and display.
#include <TimeLib.h>  //??
#include <DS1307RTC.h>
#include <OneWire.h>  // For communication with the DS18B20 temperature sensor.
#include <Spikenzie_RGBLCDShield.h>  //    For Adafruit/Spikenzie LCD
#include <utility/Spikenzie_MCP23017.h>
//#include <LiquidCrystal.h>  // For LCD   For Grove LCD
//#include <rgb_lcd.h>   // Extension for Grove LCD
#include <SPI.h>   // For microSD
#include <SD.h>
#include <PString.h> //  http://arduiniana.org/libraries/PString/
#include <Streaming.h>  // http://arduiniana.org/libraries/streaming/

// Prototype programs to demonstrate usage:
//  ChronoDot_SetTimeManuallyV2.ino
//  ClockDisplay.ino
//  GroveLogger_V1_0_DigitalTempTimeLoggingRTC.ino
//  SDExample.listfiles.ino
//

//*******************************************************************************************************************
//								                                   VARIABLE INITS
//*******************************************************************************************************************

boolean debugging = true;

// LCD display
//rgb_lcd lcd;   For Grove LCD
Spikenzie_RGBLCDShield lcd = Spikenzie_RGBLCDShield();

// Time
tmElements_t tm;  // important structure defined in Time library.
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
char fileDateName[17];
char logBuffer[60];
long logDuration;
long nextLogTime;
const long LOG_INTERVAL_MINUTES = 5;
int lastLoopSecond;
boolean startPass = true;
boolean logNewPoint = true;

const int SD_CS_Pin = 8;  // SparkFun microSD CS pin.   Most other shields CS=10;

//**************************************************************************/
//     Application specific variables.
//**************************************************************************/

//Temperature chip I/O
int DS18S20_Pin = 3;
OneWire ds(DS18S20_Pin);

float thisTempF;
float maxT = 0.0;
float minT = 1000.0;
float maxAllowedT = 200.0;  // For sanity check, degrees C.
float minAllowedT = -40.0;
float temperature;
float last_temperature;
  
/***SETUP *****************************************************************/
void setup() {
  
  Wire.begin();
  initChronoDot();
  
  Serial.begin(9600);
  // Read time from ChronoDot to make a file name for SD memory.
  readChronoDot(tm);
  PString str(fileDateName, sizeof( fileDateName)); 
  str.print((( tm.Month<10)?"0":"")); str.print(int(tm.Month));
  str.print((( tm.Day<10)?"0":"")); str.print(int(tm.Day));
  str.print((( tm.Hour<10)?"0":"")); str.print(int(tm.Hour));
  str.print((( tm.Minute<10)?"0":"")); str.print(int(tm.Minute));
  str.print(".csv");

  Serial.print( "Logging file name: ");
  Serial.println( fileDateName);
  //
  // Init: keep track of log duration.   
  logDuration = 0;
  lastLoopSecond = tm.Second;
  nextLogTime = LOG_INTERVAL_MINUTES;
   
  // Print a message to the LCD.
  lcd.begin(16, 2);  // set up the LCD's number of columns and rows
  lcd.clear();
  //         1234567890123456
  lcd.print("  LOGGER v5 ");
  lcd.setCursor(0,1);
  lcd.print(str);
  delay(5000);

  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(SD_CS_Pin, OUTPUT);
  if (!SD.begin(SD_CS_Pin)) {   // This initializes the chip select pin.   See SD.h
    Serial.println("SD initialization failed!");
    return;
  }
}  // setup


//************************************************************************

void loop() {
  // This loop inputs sensorVal every loopTime ms

  // Read time from ChronoDot
  readChronoDot(tm);  

  // Measure Temperature
  temperature = getTemp(); // if error occurred in getTemp, then return value is -1000.
  thisTempF = ((temperature * 9.0) / 5.0) + 32.0; // Convert temperature to Fahrenheit

  // Update LCD
  //clearLine(1);
  lcd.setCursor( 0, 0);
  displayTOD(tm);
  lcd.print("  ");
  lcd.print( thisTempF,1);
  clearLine(2);

  // Determine if a new point is to be logged.  
  if (lastLoopSecond > tm.Second ) {  // look for rollover of seconds
    logDuration++;                    // on rollover, increment duration counter (minutes)
    if( logDuration >= nextLogTime) { // Does the counter match the next scheduled logging time?
      logNewPoint = true;             // Yes.  Set flag.
      nextLogTime += LOG_INTERVAL_MINUTES;  // Calculate next logging time.
    }
  }
  lastLoopSecond = tm.Second;
  
  // Log a point.
  if( ( startPass == true) || (logNewPoint == true))  {// Log measurement 
    startPass = false; logNewPoint = false;
    
    // Compose measurement log string - Time and Temperature
    PString logStr(logBuffer, sizeof( logBuffer)); 
    logStr.print((( tm.Month<10)?"0":"")); logStr.print(int(tm.Month));
    logStr.print((( tm.Day<10)?"/0":"/")); logStr.print(int(tm.Day));
    logStr.print(  "/"); logStr.print(int tmYearToCalendar(tm.Year));
    logStr.print((( tm.Hour<10)?",0":",")); logStr.print(int(tm.Hour));
    logStr.print((( tm.Minute<10)?":0":":")); logStr.print(int(tm.Minute));
    logStr.print((( tm.Second<10)?":0":":")); logStr.print(int(tm.Second));
    logStr.print(",  ");
    logStr.print(logDuration);  // minutes since start
    logStr.print(",  ");
    logStr.print( thisTempF,1);
    
    // Open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    SDFile dataFile = SD.open(fileDateName, O_CREAT | O_WRITE | O_APPEND); // FILE_WRITE); 
    if (dataFile) {
      dataFile.println(logBuffer);
      Serial.println( logBuffer);
      dataFile.flush();
      dataFile.close();
      lcd.print(" Log");
    }
    else {
      Serial.println( "Error opening log file.");
      // Show status on lcd.
      clearLine(2);
      lcd.print( "Log Error");
    }
  }
    
  //delay( 1000); // Use temperature conversion time to pace this loop.
}// loop.
//*******************************************************************************************************************
//   Time                                   Functions and Subroutines
//******************************************************************************************************************* 
void initChronoDot() {
  // clear /EOSC bit
  // Sometimes necessary to ensure that the clock
  // keeps running on just battery power. Once set,
  // it shouldn't need to be reset but it's a good
  // idea to make sure.
  Wire.beginTransmission(0x68); // address DS3231
  Wire.write(0x0E); // select register
  Wire.write(0b00011100); // write register bitmap, bit 7 is /EOSC
  Wire.endTransmission();
}

void readChronoDot(tmElements_t &dateTime)
{
  //Serial.println( "This from ChronoDot.");
  RTC.read(dateTime);  // RTC declared in DS1307RTC.h
}

void showTOD(tmElements_t dateTime)
{
  //Serial.println( "This is about time.");
  print2digits(tm.Hour); Serial.print(":"); print2digits(tm.Minute); Serial.print(":"); print2digits(tm.Second); Serial.print(", ");
  Serial.print(tm.Day); Serial.print(" "); Serial.print( monthName[ tm.Month - 1]); Serial.print(" "); Serial.println( tmYearToCalendar(tm.Year));
}

void clearLine(int line) {  // line numbers 1 & 2 allowed.
  lcd.setCursor( 0, line-1);
  //         1234567890123456
  lcd.print("                ");  // 16 blank spaces.
  lcd.setCursor( 0, line-1);  
  }  
  
void displayTOD(tmElements_t &dateTime) {
  display2digits( dateTime.Hour); 
  lcd.print(":");
  display2digits( dateTime.Minute);
  lcd.print(":");
  display2digits( dateTime.Second); 
}

void displayDate( tmElements_t &dateTime) {
 lcd.print(tm.Day); 
 lcd.print(" "); 
 lcd.print( monthName[ tm.Month - 1]); 
 lcd.print(" "); 
 lcd.print( tmYearToCalendar(tm.Year));
}

void display2digits( int number) {  // 2 digits to LCD
  if( number >= 0 && number <10) {
    lcd.print('0');
  }
  lcd.print(number);
}

void print2digits(int number) {    // 2 digits to Serial.
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}
 
void append2Digits( PString buffer, int number) { // 2 digits to PString buffer
  if( number >= 0 && number <10) {
    buffer.print( "0");
  }
  buffer.print(number);
}
//*******************************************************************************************************************
//   Temperature					                         Functions and Subroutines
//*******************************************************************************************************************

//bool scanSensors() {
//  typedef byte DeviceAddress[8];
//  
//  struct thermometer{
//      DeviceAddress deviceAddress;
//      float measuredTemperatureC;
//  } thermometer[4];
//  
//  
//  
//}
float getTemp() {  //returns the temperature from one DS18B20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  delay(1000);  // Added  ajh20141125   Many examples show this delay.

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}  // getTemp()



