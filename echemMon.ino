
/* Test code for E-Chem toxic gas sensor board.
Released under Creative Commons SA-BY 4.0 license
written by Ken McGary (ken@circuitsci.com), Elliot Dicus and Peter Sand (manylabs.org)
echemMonx1  3/19/2015 Initial Release.

*/


//#define USE_MLINK // use MegunoLink PC interface
#define USE_CSV // use Comma Seperated Value output
// #define USE_TEXT  // use verbose text format output
//#define USE_HIH // use Honeywell HIH-type rH/temp sensor

#include <ArduinoTimer.h>
#include <CircularBuffer.h>
#include <CommandHandler.h>
#include <CommandProcessor.h>
#include <EEPROMStore.h>
#include <Filter.h>
//#include <MegunoLink.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <EEPROM.h>
#ifdef USE_MLINK
#include "MegunoLink.h"
// The InterfacePanel variable used to communicate with MegunoLink Pro
//InterfacePanel Panel("Test");
//TimePlot sensors;
#endif

#include "AQSettings.h"
#include "AQSensorValues.h"

/* Echem Includes */
#include <Adafruit_ADS1015.h>
#include "echem.h"

/* Settings */
AQSettings g_settings;

// This could be anything except FF (the default when fresh). This should be
// incremented if the defaults are changed.
#define VERIFICATION_CHECK_VALUE 0x01 // this should be incremented along with version number
#define DEFAULT_DATA_SET_ID 001 // this value is appended to CSV files and other output streams and can be changed via the monitor commands
#define DEFAULT_READ_INTERVAL 1000  // milliseconds between each sensor reading
#define DEFAULT_LOAD_RESISTOR 50 // AFE Rload setting in ohms, options are 10, 33, 50, 100
#define DEFAULT_OFFSET -50.0 // * 1e-9   -typical number in case cal constants aren't available
#define DEFAULT_SENSITIVITY 500.0 // * 1e-9 -typical number in case cal constants aren't available


/* Command Processing */
#define INPUT_BUFFER_LENGTH 21
#define MAX_COMMAND_ARGS 2

char g_inputBuffer[ INPUT_BUFFER_LENGTH ];
int g_inputIndex = 0;

// Commonly Used Flash Strings
#define PGMSTR(x) (__FlashStringHelper*)(x)

const char flash_menu_separator[] PROGMEM  = "# =====";
const char flash_invalid[] PROGMEM  = "Invalid: ";
const char flash_success[] PROGMEM  = "Success";

byte fetch_humidity_temperature(unsigned int *p_Humidity, unsigned int *p_Temperature);

unsigned long loopCount = 1;
unsigned long sampleDelay = 2000;  // initial value, updated by values stored in EEPROM
unsigned long time = 0;

void setup(void) 
{
  delay(5000); // wait to get serial term open
  Serial.begin(9600);  // start serial port to receive commands and send sensor data
  Wire.begin(); // initialize the I2C functions

  // We have to use this before we call analogRead, otherwise we'd short AREF to
  // the active reference voltage which could damage the microcontroller. 
  // Requires jumper from AREF to VREF (2.048V) on PX connector.
  analogReference(EXTERNAL);



#ifdef USE_CSV

  time = millis();
  while (millis()-time <15000){ // wait 15 sec for operator to get serial term open
  checkSerialCommands(); // read incoming serial commands
  }

  Serial.println(" # Hello from the echem328!");  // all comment outputs start with "# "
  Serial.println("# More info at circuitsci.com");
  Serial.print("# DataSetID = ");
  Serial.println(g_settings.dataSetId);




#ifdef USE_MLINK
  sensors.SetTitle("echem signals");  
#endif

  Serial.println(F("# Setup"));
 
  loadSettings();
  if(!settingsVerified()){
    Serial.println( F("# Using defaults") );  // start with # comment delimiter for KST and other csv-based graphing packages
    loadDefaultSettings();
  }else{
    Serial.println( F("# Settings loaded") );  // start with # comment delimiter for KST and other csv-based graphing packages
  }

  initEchem();
  Serial.print("# Init ");
  Serial.println(PGMSTR(flash_success));

  // print column headers in fourth row of csv file
  Serial.println(" ");// blank first line
  Serial.print("time");// 
  Serial.print(", vin"); 
  Serial.print(", vdd"); 
  Serial.print(", v1"); 
  Serial.print(", v2"); 
  Serial.print(", v3"); 
  Serial.print(", temp"); 
//  Serial.print(", uC"); 
Serial.print(", ppmraw"); 
Serial.println(", ppm"); 
#endif

if (g_settings.readInterval > 0) {
sampleDelay = g_settings.readInterval; // milliseconds between each sensor read operation
}


}

void loop(void) // here we go, again and again forever, until reset or something breaks.
{

time = millis();

// wait here for next sample time
//while (millis() < (time+(loopCount*sampleDelay))){
while (millis() < (time+sampleDelay)){
checkSerialCommands(); // read incoming serial commands
}
    AQSensorValues currentSensorValues;
    processSensors( currentSensorValues );
    printSensors( currentSensorValues );


loopCount++;  // increment loop conter

}



// Takes an AQSensorValues object and fills it with the current sensor values
void processSensors( AQSensorValues &sensorValues ) {

#ifdef USE_TEXT  
  Serial.println(F("Reading Sensors"));
#endif

int testanalog = analogRead(6);

  // Temp / Humidity
  #ifdef USE_HIH
    uint8_t status = readTemperatureAndHumidity( sensorValues.temperature,
      sensorValues.humidity);
    if(status){
      Serial.println( F("# HIH6121 ERR:") );
      Serial.println( status );
    }
  #endif


  // CO and diagnostic values
  readEchem( sensorValues.temperature, sensorValues.tempDelta,
    sensorValues.ppmUUTraw, sensorValues.ppmUUT, sensorValues.uutcomp,
    sensorValues.volt1diff, sensorValues.volt2diff, sensorValues.volt3 );


  // Echem voltages - 10-bit resolution, 2.048V reference, v divider ratio 7.66
  sensorValues.echemVdd = (((float)analogRead(6) / 1024.0) * 2.048) * 7.666;
  sensorValues.echemVin = (((float)analogRead(7) / 1024.0) * 2.048) * 7.666;

}


#ifdef USE_HIH
// Takes a float for temperature and humidity and fills them with the current
// values.
// Returns a status code:
// 0: Normal
// 1: Stale - this data has already been read
// 2: Command Mode - the sensor is in command mode
// 3: Diagnostic - The sensor has had a diagnostic condition and data is
//    invalid
// 4: Invalid - The sensor did not return 4 bytes of data. Usually this means
//    it's not attached.
uint8_t readTemperatureAndHumidity( float &temperature_c, float &humidity_percent ) {
  // From: http://www.phanderson.com/arduino/hih6130.html
  uint8_t address = 0x27;
  uint8_t humHigh, humLow, tempHigh, tempLow, status;
  uint16_t humData, tempData;

  // Request read
  Wire.beginTransmission(address);
  Wire.endTransmission();

  // According to the data sheet, the measurement cycle is typically ~36.56 ms
  // We'll give a little extra time

  // Request data
  uint8_t bytesReceived = Wire.requestFrom((int)address, (int)4);
  if(bytesReceived != 4){

    // This is our own error to specify that we didn't receive 4 bytes from the
    // sensor.
    return 4; // temp and humidity will be unchanged
  }

  humHigh  = Wire.read();
  humLow   = Wire.read();
  tempHigh = Wire.read();
  tempLow  = Wire.read();

  // Status is the top two bits of the high humidity byte
  status = (humHigh >> 6) & 0x03;
  if(status == 3){
    return status; // temp and humidity will be unchanged
  }

  // Keep the rest
  humHigh = humHigh & 0x3f;

  // OR in the low bytes
  humData  = (((uint16_t)humHigh)  << 8) | humLow;
  tempData = (((uint16_t)tempHigh) << 8) | tempLow;

  // The bottom two bits of the low temp byte are invalid, so we'll remove
  // those
  tempData = tempData >> 2;

  // Convert to floating point
  humidity_percent = (float) humData * 6.10e-3; // 100 / (2^14 - 1)
  temperature_c = (float) tempData * 1.007e-2 - 40.0; // 165 / (2^14 - 1)

  // Status can be
  // 0: Normal
  // 1: Stale - this data has already been read
  // 2: Command Mode - the sensor is in command mode
  // 3: Diagnostic - The sensor has had a diagnostic condition and data is
  //    invalid
  return status;
}
#endif


// Takes an AQSensorValues object. Print the sensor values, 1 per line, to the
// serial port
void printSensors( const AQSensorValues &sensorValues ) {
#ifdef USE_CSV
// print raw a/d values and fixed values to serial port
  Serial.print(loopCount);// Serial.print(", ");Serial.print(volt0,2);
  Serial.print(", "); Serial.print(sensorValues.echemVin,2);
  Serial.print(", "); Serial.print(sensorValues.echemVdd,2);
  Serial.print(", "); Serial.print(sensorValues.volt1diff,2); // aux raw a/d signal
  Serial.print(", "); Serial.print(sensorValues.volt2diff,2); // main CO raw a/d signal
  Serial.print(", "); Serial.print(sensorValues.volt3,2); // AFE internal reference (half of board ref or 1.024V)
//  Serial.print(", "); Serial.print(chiptemp,2);
  Serial.print(", "); Serial.print(sensorValues.tempDelta,2); // tempDelta is holdover from AQ Station code, is actually xtemp here or echem stemp sensor value as read from a/d channel 0 
//  Serial.print(", "); Serial.print(sensorValues.uutcomp,8);//
//  Serial.print(", "); Serial.print(auxGainFactor,2);
//  Serial.print(",R "); Serial.print(RH,1);
//  Serial.print(",T "); Serial.print(T_C,1);
//  Serial.print(", "); Serial.print(testanalog);
  Serial.print(", "); Serial.print(sensorValues.ppmUUTraw,4);
  Serial.print(", "); Serial.println(sensorValues.ppmUUT,4);
#endif



#ifdef USE_MLINK
// MegunoLink stuff here
Serial.print("{TIMEPLOT|data|ppm_raw|T|");
Serial.print(ppmUUTraw,4);
Serial.println("}");
Serial.print("{TIMEPLOT|data|temp|T|");
Serial.print(xtemp,2);
Serial.println("}");
Serial.print("{TIMEPLOT|data|auxcomp|T|");
Serial.print(auxcomp,2);
Serial.println("}");
//sensors.SendData("ppm_raw",ppmUUTraw);
#endif

#ifdef USE_TEXT
  Serial.print( F("Temp: ") );
  Serial.println(sensorValues.temperature, 2);

  Serial.print( F("Humid: ") );
  Serial.println(sensorValues.humidity, 2);

  Serial.print( F("CO: ") );
  Serial.println(sensorValues.ppmUUT, 2);

  Serial.print(F("eVdd: ") );
  Serial.println(sensorValues.echemVdd, 4);

  Serial.print(F("eVin: ") );
  Serial.println(sensorValues.echemVin, 4);

  Serial.print(F("ppmRaw: ") );
  Serial.println(sensorValues.ppmUUTraw, 4);

  Serial.print(F("ppm: ") );
  Serial.println(sensorValues.ppmUUT, 4);

  Serial.print(F("uutComp: ") );
  Serial.println(sensorValues.uutcomp, 8);

  Serial.print(F("v1diff: ") );
  Serial.println(sensorValues.volt1diff, 4);

  Serial.print(F("v2diff: ") );
  Serial.println(sensorValues.volt2diff, 4);

  Serial.print(F("v3: ") );
  Serial.println(sensorValues.volt3, 4);

  Serial.print(F("tempDel: ") );
  Serial.println(sensorValues.tempDelta, 2);
#endif
}

// Prints the current calibration/settings values
void printCalibration() {
  Serial.print( F("# version: ") );
  Serial.println(g_settings.verification_check, HEX);

  Serial.print( F("# dataSetId: ") );
  Serial.println(g_settings.dataSetId);

  Serial.print( F("# readInterval: ") );
  Serial.println(g_settings.readInterval);

  Serial.print( F("# loadResistor: ") );
  Serial.println(g_settings.loadResistor);

  Serial.print( F("# sens: ") );
  Serial.println(g_settings.uutSensitivity, 2);

  Serial.print( F("# offset: ") );
  Serial.println(g_settings.uutOffset, 2);
}

// get index of given character; return -1 if not found
// fix(smaller): replace with strchr
inline int indexOf( const char *str, char c ) {
  int index = 0;
  while (true) {
    char s = *str++;
    if (s == c)
      return index;
    if (s == 0)
      break;
    index++;
  }
  return -1;
}

// execute an incoming serial command
boolean executeCommand( const char *command, byte argCount, char *args[] ) {

  Serial.println( PGMSTR(flash_menu_separator) );
  if( command[0] == '0' ){ // Show Menu


    Serial.print(F("# Last Status: "));
    printMenu();
    return true;

  }else if( command[0] == '1' ){ // Current values

    AQSensorValues currentSensorValues;
    processSensors( currentSensorValues );
    Serial.print("# ");
    printSensors( currentSensorValues );
    return true;

  }else if( command[0] == '2' ){ // Settings

    printCalibration();
    return true;

  }else if( command[0] == '3' ){ // Load default settings
    if(argCount == 0){
      Serial.println( F("# Submit \"3:confirm\"") );
    }else if( argCount == 1 && strcmp_P(args[0], PSTR("confirm")) == 0 ){
      loadDefaultSettings();
    }
    return true;

  }else if( command[0] == '4' ){ // Edit settings

    if(argCount == 0){
      Serial.println( F("# Submit \"4:<number>,<value>\"") );
      Serial.println( F("# 0:datasetId") );
      Serial.println( F("# 1:readInterval") );
      Serial.println( F("# 2:loadResistor") );
      Serial.println( F("# 3:sens") );
      Serial.println( F("# 4:offset") );
    }else if(argCount == 2){
      // Check for the value to edit
      Serial.print("# "); // make it an output comment
      switch( atoi(args[0]) ) {
        case 0:
          g_settings.dataSetId = atoi(args[1]);
          Serial.println( g_settings.dataSetId );
          break;
        case 1:
          g_settings.readInterval = atoi(args[1]);
          Serial.println( g_settings.readInterval );
          break;
        case 2:
          g_settings.loadResistor = atoi(args[1]);
          Serial.println( g_settings.loadResistor );
          break;
        case 3:
          g_settings.uutSensitivity = atof(args[1]);
          Serial.println( g_settings.uutSensitivity, 2 );
          break;
        case 4:
          g_settings.uutOffset = atof(args[1]);
          Serial.println( g_settings.uutOffset, 2 );
          break;
        default:
          Serial.print( PGMSTR(flash_invalid) );
          Serial.println(command);
          break;
      }
      saveSettings();
    }
    return true;

  }

  // Any invalid options
  Serial.print( PGMSTR(flash_invalid) );
  Serial.println(command);
  printMenu();
  Serial.println( PGMSTR(flash_menu_separator) );
  return false;
}

// Prints the menu
void printMenu() {
  Serial.println( F("0: Menu") );
  Serial.println( F("1: Values") );
  Serial.println( F("2: Settings") );
  Serial.println( F("3: Load Defaults") );
  Serial.println( F("4: Edit Settings") );
}

// Parse the incoming serial buffer and execute any command it contains
void processMessage() {
  g_inputBuffer[ g_inputIndex ] = 0; // truncate the string
  g_inputIndex = 0;
  char *command = g_inputBuffer;
  char *args[ MAX_COMMAND_ARGS ];
  int argCount = 0;

  // check for command arguments
  int colonPos = indexOf( g_inputBuffer, ':' );
  if (colonPos > 0) {
    g_inputBuffer[ colonPos ] = 0;
    char *argStr = g_inputBuffer + colonPos + 1;
    int commaPos = 0;
    argCount = 0;
    do {

      // strip leading spaces
      while (*argStr == ' ')
        argStr++;

      // store in argument array
      args[ argCount ] = argStr;

      // find end of arg
      commaPos = indexOf( argStr, ',' );
      char *argEnd = 0;
      if (commaPos > 0) {
        argStr[ commaPos ] = 0;
        argStr += commaPos + 1;
        argEnd = argStr - 2;
      } else {
        argEnd = argStr + strlen( argStr ) - 1;
      }

      // strip off tailing spaces (note: we're protected by 0 that replaced
      // colon)
      while (*argEnd == ' ') {
        *argEnd = 0;
        argEnd--;
      }

      // done with this arg
      argCount++;
    } while (commaPos > 0 && argCount < MAX_COMMAND_ARGS);
  }

  // execute the command now that we have parsed it
  if (command[ 0 ]) {
      boolean handled = executeCommand( command, argCount, args );
  }
}


// Process a single incoming command byte; store in buffer or execute command if
// complete
void feedCommandByte( char c ) {
  if (c == 10 || c == 13) {
    if (g_inputIndex)
      processMessage();

  // want to be able to write 0 into last position after increment
  } else if (g_inputIndex >= INPUT_BUFFER_LENGTH - 1) {
    g_inputIndex = 0;
  } else {
    g_inputBuffer[ g_inputIndex ] = c;
    g_inputIndex++;
  }
}


// Read incoming serial commands; will execute any received commands
void checkSerialCommands() {
  while (Serial.available()) {
//    wdt_reset();
    char c = Serial.read();
    if (c > 0) {
      feedCommandByte( c );
    }
  }
}

// Load the default settings and save them to the eeprom
void loadDefaultSettings() {
  g_settings.verification_check = VERIFICATION_CHECK_VALUE;
  g_settings.dataSetId      = DEFAULT_DATA_SET_ID;
  g_settings.readInterval   = DEFAULT_READ_INTERVAL; // milliseconds between readings (1-1000)
  g_settings.loadResistor   = DEFAULT_LOAD_RESISTOR; // AFE Rload setting in ohms, options are 10, 33, 50, 100
  g_settings.uutSensitivity = DEFAULT_SENSITIVITY; // * 1e-9
  g_settings.uutOffset      = DEFAULT_OFFSET; // * 1e-9
  saveSettings();
}

// Returns true if the current settings match the verification check value and
// both the postInterval and diagInteval are > 0. False otherwise.
bool settingsVerified() {
  // Check the verification value and some small sanity checks.
  if(
    g_settings.verification_check == VERIFICATION_CHECK_VALUE &&
    g_settings.readInterval > 0 &&
    g_settings.loadResistor > 0
  ){
    return true;
  }
  return false;
}

// Load settings from eeprom
int loadSettings() {
  Serial.println( F("# Load") );
  int address = 0;
  uint8_t* p = (uint8_t*)(void*)&g_settings;
  uint16_t i;
  for (i = 0; i < sizeof(g_settings); i++){
//    wdt_reset(); // Reset the watchdog timer
    *p++ = EEPROM.read(address++);
  }
  Serial.print("# Load ");
  Serial.println( PGMSTR(flash_success) );
  return i;
}

// Save settings back to eeprom
int saveSettings() {
  Serial.println( F("# Save") );
  int address = 0;
  const uint8_t* p = (const uint8_t*)(const void*)&g_settings;
  uint16_t i;
  for(i = 0; i < sizeof(g_settings); i++){
//    wdt_reset(); // Reset the watchdog timer
    EEPROM.write(address++, *p++);
  }
  Serial.print("# Save ");
  Serial.println( PGMSTR(flash_success) );
  return i;
}
