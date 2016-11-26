
/*
  The purpose of this code is to measure temperature, display and record these values to SD memory.

 Created 22 Nov 2014
 Modified 22Nov2014
 by Alan Hutchenreuther

 Temperature sensor is the DS18B20, with digital output to the MCU over the OneWire interface.

 The display portion of this code is adapted from I2C_LCD_Demo from Spikenzie Labs
 The Arduino-LCD interface wire count is reduced by using the MCP23017 with I2C interface as a port expander and
 the Two Wire Interface on the Arduino (TWI library).

 LCD controller data sheets apply:
 Ref 1:   Hitachi HD44780U (LCD-II)  8 x 2 lines.
 Ref 2:   Specifications of LCD Module (ADM1602K-NSW-FBS/3.3V) using ST7066 controller or equivalent.
 Ref 3:   Specification for LCD Module (BC-1602H-SAGA-N-B-A00)

 The data logging function is done using the microSD memory function that is built into the Arduino Ethernet SHIELD.

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is part of the public domain

 DigitalTempTimeLogging V6

 Modified 22-25 Nov 2014
 - Serial interface is used to display program timing.
 - Other parts of this code is adapted from TemperatureControl_MSP430_20Oct V4.1 for the MSP430
 and DataloggerWithTemperatureControl for the Arduino.  Temperature measurement is also
 adapted from SD card datalogger

 modified 28 Nov 2014  (21,750 bytes of 32,256 bytes max)
 - Add Ethernet shield to get time from network time protocol, NTP, time server.
 Code lifted from UdpNtpClient_AJHEthernet.ino
 - 24-hour time is displayed on LCD.
 - debugging option allows display of key variables.

 Modified 28Nov14 (31,276 bytes of 32,256 bytes max)
 - Add logging of temperature and time to SD memory.   SD code lifted from DataloggerWithTempertureControl.ino
 - Add println() at end of SD output to add CRLF.
 - comment out debugging option code to make this version fit in 32,256 bytes.

 Issues:
 1) Occasional hang on start-up after putting Temperature Logger v7 on LCD.   NEW since last mod.
 2) Time logged is a few seconds off due to when elapsed time is calculated.

 DigitalTempTimeLogging V7
 Modified 2Dec2014
 - Recover program space
 Convert LCD display back to parallel, 4-bit style.
 - Restore debugging option code (size 31,464 bytes)


 DigitalTempTimeLoggingRTC V8
Modified 4Dec14 (approx) (31,698 bytes of 32,256 bytes max)
Change setup to initialize SD memory before using ethernet interface.
This seemed to fix occasional hang on startup (issue #1 above)and message 'Failed to configure Ethernet using DHCP'
Fixed V7 issue:  "Occasional hang on start-up after putting Temperature Logger v7 on LCD.   NEW since last mod."

7Dec14
- Add ChronoDot as Real-time clock.
  Step 1: Comment out Ethernet function to load software RTC to save program space.
  Remove Ethernet Function Option Parts A-E and # includes (20,936 bytes approx)
  Step 2: Add ChronoDot time Read-out to LCD  (22,082 bytes approx)
  This time base is more stable and there is no elapsed time calculation needed (remove Issue #2 above)
16Dec14 (23,010 bytes of 32,256 bytes max)
-  Log temperature and time once per minute.   17 bytes/entry.  Approx capacity 171 years in 2GB microSD chip??
-  Log new date on first tick after midnight
-  Repackaged hardware.
18Dec14 (23,022 bytes)
- Write 'Log' to LCD and date to microSD immediately as a diagnostic indicator that memory works.
  If memory not present, or datafile not opened, write "Err".
- Eliminate seconds portion in logged data.... always zero!

11Feb2015 version 8.2 (23,274 bytes)  // ajh20150210
- To make test for new minute more robust, look for t_seconds to drop in value instead of equaling zero.
- Do a sanity check on temperature.   If out of range, force temp to repeat last valid measurement.
- Logged time: put leading zero in log when t_minutes < 10.

28Feb2015 version 8.3
- Switch data order to put Time before temperature to simplify Excel plotting
- Increase log interval to attempt to eliminate skipped logs at 1 minute rate,
  also to reduce data volume when desired.
  Tested at 2 minute interval.
  Set to 5 minutes for outdoor temp monitor, 8Mar15, LOGSD.TXT
- Conditional compiles depending on shield selection.  (IN PROCESS)
  a) Arduino Ethernet shield vs SparkFun microSD shield. DONE - MICRO_SD_SHIELD
  b) LCD display vs Color TFT touch sensitive display.
- MICRO_SD_SHIELD case: remove all LCD displays.
14Apr15
- Add yellow LED to indicate sd status: log (.5 sec blink) or error (solid ON).


Operating Note:  When swapping out microSD cards, remove power.   Failure to do so will
prevent future logging to continue without error indication.

*/

// include the library code:
// See Reference for Libraries.Contributed Libraries for instructions.
// Putting library in the search path allows short reference using <filename> syntax.
#include <OneWire.h>  // For communication with the DS18B20 temperature sensor.
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>
// For ChronoDot RTC Option
#include <Wire.h>

// Hardware Configurations
#define MICRO_SD_SHIELD  // Uses when SparkFun microSD shield is present, to configure chip select.

//*******************************************************************************************************************
//								                                   VARIABLE INITS
//*******************************************************************************************************************

boolean debugging = true;

// initialize the library with the numbers of the interface pins
//               RS, Enable, D4, D5, D6, D7
LiquidCrystal lcd(3, 5, 6, 7, 10, 9);

float thisTempF = 0.0;
float maxT = 0.0;
float minT = 1000.0;
float maxAllowedT = 200.0;  // For sanity check, degrees C.
float minAllowedT = -40.0;
float temperature;
float last_temperature;

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
boolean startPass = true;
boolean logNewPoint = true;
const long LOG_INTERVAL_MINUTES = 5;

boolean new_minute;
unsigned long t_minutes_previous;
unsigned long t_seconds_previous;

unsigned long t_hours;
unsigned long t_minutes;
unsigned long t_seconds;
unsigned long t_day;
unsigned long t_date;
unsigned long t_month;
unsigned long t_monthinfo;
unsigned long t_year;

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

//Temperature chip I/O
OneWire ds(DS18S20_Pin); // on digital pin 2

// SD Memory variables.
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
#if defined( MICRO_SD_SHIELD)
/*
SparkFun microSD shield
Communication with microSD cards is achieved over an SPI interface.
The SCK, DI, and DO pins of the microSD socket are broken out to the
ATmega168/328’s standard SPI pins (digital 11-13), while the CS pin is
broken out to Arduino’s D8 pin. If you decide to use one of the many
open source FAT libraries (like FAT16 or SDFat) make sure to change
the code to reflect the location of the CS pin.

Most libraries assume the CS pin is connected to D10; this will have to
be changed to D8. Also for the libraries to work pin D8 will have to be
set as an output in the ‘setup()’ section of your sketch.
*/
const int chipSelect = 8;
const char logFile[20] = "LogSD.txt";
// Status indication in absence of display.  
boolean ledState = true;  
int ledPin = 4;  // Default pin 13 not for use with SparkFun microSD shield.  13 is SPI SCLK.
#else
/*
Arduino Ethernet shield
*/
const int chipSelect = 4;
const char logFile[20] = "Log.txt";
#endif


/***SETUP *****************************************************************/
void setup() {
  pinMode( ledPin, OUTPUT);
  
  Wire.begin();
  Serial.begin(9600);

  // Print a message to the LCD.
  lcd.begin(16, 2);  // set up the LCD's number of columns and rows
  lcd.print("Temperature");
  lcd.setCursor(0, 1);
  lcd.print("   Logger v8.3");


  // see if the card is present and can be initialized:
  // This needs to be done BEFORE starting Ethernet or else will get
  // 'Failed to configure Ethernet using DHCP' if SD card is plugged in.
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    digitalWrite( ledPin, HIGH);
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  // ChronoDot Readout Function, Part A
  // clear /EOSC bit
  // Sometimes necessary to ensure that the clock
  // keeps running on just battery power. Once set,
  // it shouldn't need to be reset but it's a good
  // idea to make sure.
  Wire.beginTransmission(0x68); // address DS3231
  Wire.write(0x0E); // select register
  Wire.write(0b00011100); // write register bitmap, bit 7 is /EOSC
  Wire.endTransmission();
  // ChronoDot Readout Function, Part A

   
}  // setup


//************************************************************************

void loop() {
  // This loop inputs sensorVal every loopTime ms

  char strThisTempF[16];
  char strHours[5];
  char strMinutes[5];
  char strSeconds[5];

  char strMaxT[16];
  char strMinT[16];


  /**** ChronoDot Time, Part B **********/
  // send request to receive data starting at register 0
  Wire.beginTransmission(0x68); // 0x68 is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(0x68, 7); // request three bytes (seconds, minutes, hours)
  // plus 4 bytes for date (day, date, month #, year)

  while (Wire.available())
  {
    t_seconds_previous = t_seconds; // For more robust test for a new minute. It is possible that t_seconds may skip the '0' value occasionally.
    t_seconds = Wire.read(); // get seconds
    t_minutes = Wire.read(); // get minutes
    t_hours = Wire.read();   // get hours

    t_seconds = (((t_seconds & 0b11110000) >> 4) * 10 + (t_seconds & 0b00001111)); // convert BCD to decimal
    t_minutes = (((t_minutes & 0b11110000) >> 4) * 10 + (t_minutes & 0b00001111)); // convert BCD to decimal
    t_hours = (((t_hours & 0b00100000) >> 5) * 20 + ((t_hours & 0b00010000) >> 4) * 10 + (t_hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)
    if ( startPass) {
      t_minutes_previous = t_minutes;
    }
    if ( logNewPoint) {
      Serial.print(t_hours); Serial.print(":"); Serial.print(t_minutes); Serial.print(":"); Serial.print(t_seconds); Serial.print(", ");
    }
    t_day = Wire.read();   // get day number
    t_date = Wire.read();  // get date
    t_monthinfo = Wire.read(); // get month, and century (0 or 1)
    t_year = Wire.read();  // get year 0-99 in century.

    t_day = t_day & 0b00000111;  // motion to convert BCD to decimal.
    t_date = (((t_date & 0b11110000) >> 4) * 10 + (t_date & 0b00001111)); // convert BCD to decimal
    t_month = ((( t_monthinfo & 0b00010000) >> 4) * 10 + (t_monthinfo & 0b00001111));
    t_year = ((( t_year & 0b11110000) >> 4) * 10 + (t_year & 0b00001111)); // convert BCD to decimal
    t_year = t_year + ((t_monthinfo & 0b10000000) >> 7) * 100 + 2000;

    //Serial.print( t_day); Serial.print("/");
    if ( logNewPoint) {
      Serial.print(t_date); Serial.print(" "); Serial.print( monthName[ t_month - 1]); Serial.print(" "); Serial.println( t_year);
    }
  }

  /**** ChronoDot Time, Part B **********/

  /***** Temperature*******/
  // Read digital temperature sensor DS18B20
  last_temperature = temperature;  // ajh20150210
  temperature = getTemp();
  if ((temperature > maxAllowedT) || (temperature < minAllowedT)) {
    temperature = last_temperature;  // through out erroneous reading.
  }

  // loopTime = 34 ms with no delay() in getTemp();
  // loopTime = 1,034 ms with delay(1000) in getTemp();  No surprise.

  // Convert temperature to Fahrenheit
  thisTempF = ((temperature * 9.0) / 5.0) + 32.0;

  // Record Max and Min T for display purposes
  if (thisTempF > maxT) {
    maxT = thisTempF;
  }
  else if (thisTempF < minT) {
    minT = thisTempF;
  }
  //// Ethernet Function Option, Part D
  //// Calculate current UNIX time and local time to the nearest second.
  //  current_time = lastPolledTime + (millis() - sketchTime)/1000; // millis() wraps after about 50 days!
  //  t_hours = ((current_time % 86400L)/ 3600 );
  //  t_minutes = ((current_time % 3600) / 60);
  //  t_seconds = (current_time % 60);
  //// Ethernet Function Option, Part D

  // Display Key Values .print function can handle floats with 1 decimal place.
//  dtostrf( thisTempF, 5, 1, strThisTempF);
//  dtostrf( maxT, 5, 1, strMaxT);
//  dtostrf( minT, 5, 1, strMinT);
//  dtostrf( t_hours, 2, 0, strHours);
//  dtostrf( t_minutes, 2, 0, strMinutes);
//  dtostrf( t_seconds, 2, 0, strSeconds);

  /*  Code for standard interface to basic LCD.   */
  lcd.clear();
  lcd.print(  strThisTempF) ;
  lcd.print( "  ");
  lcd.print( t_hours); /* strHours);*/
  lcd.print(":");
  if ( t_minutes < 10 ) {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    lcd.print("0");
  }
  lcd.print( t_minutes) /* strMinutes)*/ ;
  lcd.print(":");
  if ( t_seconds < 10 ) {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    lcd.print("0");
  }
  lcd.print( t_seconds);  //strSeconds);

  lcd.setCursor(0, 1);
  lcd.print( strMinT);
  lcd.print( ",");
  lcd.print( strMaxT);

  //// Ethernet Function Option, Part E
  //// loopTime = 1156 approx, with debugging = false;
  //  loopTime = millis() - loopTime;
  //  secondsSinceStart = secondsSinceStart + (float)loopTime/1000.0;
  //  if( debugging) {
  //    if( count < 10) {
  //      Serial.print(count);
  //      Serial.print(", ");
  //      Serial.print( loopTime);
  //      Serial.print(", ");
  //      Serial.print( secondsSinceStart);
  //      Serial.print("  ");
  //      Serial.print(t_hours);
  //      Serial.print(":");
  //      Serial.print(t_minutes);
  //      Serial.print(":");
  //      Serial.println(t_seconds);
  //      count += 1;
  //    }
  //  }
  //// Ethernet Function Option, Part E

  // Detect new minute occurrence on clock.
  //  if( t_seconds < t_seconds_previous) // ajh20150210
  //    new_minute = true;
  //    logNewPoint = true;
  //  else
  //    new_minute = false;
  //    logNewPoint = false;
  //
  // Detect completion of log interval.
  logNewPoint = false;
  if ( (t_minutes - t_minutes_previous) >= LOG_INTERVAL_MINUTES) {
    logNewPoint = true;
    t_minutes_previous = t_minutes;
  }

  // Record measurement to data log.
  if (  (logNewPoint == true) || (startPass == true)) {

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    // SD.open(),dataFile.flush() calls modified/added to speed up code, per forum.
    // Search forum for "Why is the SD library slow?"
    File dataFile = SD.open( logFile, O_CREAT | O_WRITE | O_APPEND); // Options per sdfatlib described in forum.

    // if the file is available, write to it:
    if (dataFile) {
      if ((( t_hours == 0) & (t_minutes == 0) & (logNewPoint == true)) || (startPass == true)) { // Update date after midnight or on Start-up.
        dataFile.print( ",,"); dataFile.print(t_date); dataFile.print(" "); dataFile.print( monthName[ t_month - 1]); dataFile.print(" "); dataFile.println( t_year);
        startPass = false;  // diagnostic write of date to microSD.  'Log' stays on LCD if there is an error.
      }

      dataFile.print( strHours);
      dataFile.write( ":");
      // strMinutes may be " n" or "nn".   Need to replace space with '0' if minutes < 10.
      if ( t_minutes < 10) { // ajh20150210
        //dataFile.print( "0");
        strMinutes[0] = '0';
      }
      dataFile.print( strMinutes);
      //dataFile.write( ":");
      //dataFile.write( strSeconds);
      dataFile.write(", ");
      dataFile.print( strThisTempF);
      dataFile.println(); // CRLF
      dataFile.flush();
      dataFile.close();
      // Indicate a point has been logged.
      lcd.print( " Log");
      Serial.println("      Logging");
#if defined( MICRO_SD_SHIELD)
      digitalWrite( ledPin, HIGH);  // Blink on successful log.
      delay( 500);
      digitalWrite( ledPin, LOW);
#endif
    }
    else {
      lcd.print( " Err");
      digitalWrite(ledPin, HIGH);  // Announce error if there is a problem opening the data file.
    }
  }

}// loop.



//*******************************************************************************************************************
//   Temperature					                         Functions and Subroutines
//*******************************************************************************************************************

float getTemp() {
  //returns the temperature from one DS18B20 in DEG Celsius

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




