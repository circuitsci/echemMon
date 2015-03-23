#ifndef _ECHEM_
#define _ECHEM_

#define USE_XTEMP

// #define USE_TESTPRINT
#define TEMPERATURE_SCALE_FACTOR 0.3

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Wire.h>
#include "AQSettings.h"
#include "LMP91000.h"
// #include <Adafruit_ADS1015.h> // This is included in MainAQ.ino

// Echem Settings
#define SETTLE_DELAY 100
#define AVG_NUM      32
#define AVG_DELAY    1
#define TIA          200.0    // in Kohms
#define TIA_X        200.0    // in Kohms

/* Use this for the 16-bit version, must set jumper to 49 or higher, AFE is
hardwired to 48 */
Adafruit_ADS1115 ads(0x49);

// Interpolation tables
#define SENSITIVITY_TABLE_LENGTH 9
float sensitivity_temperature_values[ SENSITIVITY_TABLE_LENGTH ] = {
  -30, -20, -10, 0, 10, 20, 30, 40, 50 };

float sensitivity_factor_values[ SENSITIVITY_TABLE_LENGTH ] = {
  1.92, 1.69, 1.43, 1.22, 1.09, 1.00, 0.93, 0.89, 0.86 };

#define ZERO_CURRENT_TABLE_LENGTH 7
float zero_current_temperature_values[ ZERO_CURRENT_TABLE_LENGTH ] = {
  -10, 0, 10, 20, 30, 40, 50 };

float zero_current_factor_values[ ZERO_CURRENT_TABLE_LENGTH ] = {
  5, 0, -5, -40, -95, -205, -375 };

// map a value in the fromTable to a value in the toTable,
// linearly interpolating between table entries;
// assumes both tables have same length (specified by length);
// assumes fromTable values are increasing;
// clips above and below the fromTable table
float interpolate( float value, float *fromTable, float *toTable, int length ) {
  float result = NAN;
  if (value <= fromTable[ 0 ]) {
    result = toTable[ 0 ];
  } else if (value >= fromTable[ length - 1 ]) {
    result = toTable[ length - 1 ];
  } else {
    for (int i = 0; i < length - 1; i++) {
      if (value >= fromTable[ i ] && value <= fromTable[ i + 1 ]) {
        float frac = (value - fromTable[ i ]) / (fromTable[ i + 1 ] - fromTable[ i ]);
        result = toTable[ i ] * (1 - frac) + toTable[ i + 1 ] * frac;
        break;
      }
    }
  }
  return result;
}

void initEchem(){
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  Wire.begin(); // initialize the I2C functions
  ads.begin();

  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);           // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV


  LMP_CFG(); //Configure the LMP91000 AFE device


}

void readEchem(const float tempC, float &tempDelta, float &ppmUUTraw, float &ppmUUT,
  float &uutcomp, float &volt1diff, float &volt2diff, float &volt3) {

//  wdt_reset(); // reset the watchdog timer
  delay(SETTLE_DELAY);

  double ads_bitmv = 0.125; // since we're using GAIN_ONE in initEchem

  // xtemp sensor output is low noise low impedance and we don't need much
  // precision so doesn't need averaging.
  int16_t adc0 = ads.readADC_SingleEnded(0);
  double volt0 = adc0 * ads_bitmv;

  volt1diff = 0;
  volt2diff = 0;
  volt3 = 0;

  //This averaging loop could be improved I'm sure...
  for (int avgLoop = 0; avgLoop < AVG_NUM; avgLoop++){
//    wdt_reset(); // reset the watchdog timer

    // channel 3 reads on-board 2.048V voltage reference divided by 2 and
    // buffered by LMP91000, is half-way bias point for echem sensor
    int16_t adc3 = ads.readADC_SingleEnded(3);

    // main sensor signal compared to half-way point
    int16_t adc2diff = ads.readADC_Differential_2_3();

    // aux sensor signal compared to half-way point
    int16_t adc1diff = ads.readADC_Differential_1_3();

    volt2diff = volt2diff + (adc2diff * ads_bitmv);
    volt1diff = volt1diff + (adc1diff * ads_bitmv);
    volt3 = volt3 + (adc3 * ads_bitmv);

    delay(AVG_DELAY);
  }

  volt1diff = volt1diff / AVG_NUM;
  volt2diff = volt2diff / AVG_NUM;
  volt3 = volt3 / AVG_NUM;

  float xtemp = (volt0 - 500) / 10;


  tempDelta = xtemp - tempC;

  float temp_calc = tempC + (tempDelta * TEMPERATURE_SCALE_FACTOR);

  #ifdef USE_XTEMP
    temp_calc = xtemp;
  #endif


  float sensitivity_factor = interpolate( temp_calc,
    sensitivity_temperature_values, sensitivity_factor_values,
    SENSITIVITY_TABLE_LENGTH );

  float zero_current_factor = interpolate( temp_calc,
    zero_current_temperature_values, zero_current_factor_values,
    ZERO_CURRENT_TABLE_LENGTH );

  // zero_current_factor is in nanoamps convert to microamps here
  float zero_current_factor_ua = zero_current_factor / 1000;

  float cgain_per_ppm = g_settings.uutSensitivity * 1e-3; // microamps per ppm

  uutcomp = ( volt2diff  / TIA); // Working Electrode current in microamps (millivolts / Kohms)

  // a rough value with no correction factors considered
  ppmUUTraw = volt2diff / (cgain_per_ppm * TIA); // Aux Electrode current in milliamps

  ppmUUT = ((uutcomp - zero_current_factor_ua) * sensitivity_factor) / cgain_per_ppm;

  #ifdef USE_TESTPRINT
    // Serial.print( F("TIA: ") );
    // Serial.println(TIA, 8);
    // Serial.print( F("TIA_X: ") );
    // Serial.println(TIA_X, 8);
    Serial.print( F("cgain: ") );
    Serial.println(cgain_per_ppm, 8);
    Serial.print( F("uutcomp: ") );
    Serial.println(uutcomp, 8);
    Serial.print( F("zerocomp: ") );
    Serial.println(zero_current_factor_ua, 8);
    Serial.print( F("uS: ") );
    // Serial.println(g_settings.uutSensitivity, 8);
    // Serial.print( F("uGF: ") );
    // Serial.println(uutGainFactor, 8);
    Serial.print( F("uSF: ") );
    Serial.println(sensitivity_factor, 8);
  #endif
}

#endif //_ECHEM_
