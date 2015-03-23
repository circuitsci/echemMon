#ifndef _LMP91000_
#define _LMP91000_

/*
LM91000.h header info originally created by: Sindre Søpstad
*/

#define LMP91000_Slave_Address 0x48   //hardwired in chip, only one I2C address with MENB enable pin.

// internal temp sensor calculations - plug in your max and min parameters for
// this two-point interpolation, less error with smaller temp range
// see table in datasheet
#define LMP91000_V2 1154.0 // TEMP_MAX_MV from LMP91000 temp data table 50C
#define LMP91000_T2 50.0   // TEMP_MAX in degC
#define LMP91000_V1 1480.0 // TEMP_MIN_MV from LMP91000 temp data table 10C
#define LMP91000_T1 10.0   // TEMP_MIN in degC


#define STATUS 0x00
#define LOCKCN 0x01

#define TIACN 0x10
#define REFCN 0x11
#define MODECN 0x12

#define Fet_Short_Enabled 0x80
#define Deep_sleep 0x00
#define Two_lead_gnd 0x01
#define Standby 0x02
#define Three_Lead_Amperometric 0x03
#define Temperature_meas_TIA_Off 0x06
#define Temperature_meas_TIA_On 0x07

#define Write_TIACN_REFCN 0x00
#define Read_TIACN_REFCN 0x01

#define Gain_External_Resistance 0x00
#define Gain_2p75KOhm 0x04
#define Gain_3p5KOhm 0x08
#define Gain_7KOhm 0x0C
#define Gain_14KOhm 0x10
#define Gain_35KOhm 0x14
#define Gain_120KOhm 0x18
#define Gain_350KOhm 0x1C

#define Rload_10_Ohm 0x00
#define Rload_33_Ohm 0x01
#define Rload_50_Ohm 0x02
#define Rload_100_Ohm 0x03

#define Internal_Vref 0x00
#define External_Vref 0x80

#define Internal_Zero_20 0x00
#define Internal_Zero_50 0x20
#define Internal_Zero_67 0x40
#define Internal_Zero_Bypassed 0x60

#define Bias_Negative 0x00
#define Bias_Positive 0x10

#define Bias_0_Percent 0x00
#define Bias_1_Percent 0x01
#define Bias_2_Percent 0x02
#define Bias_4_Percent 0x03
#define Bias_6_Percent 0x04
#define Bias_8_Percent 0x05
#define Bias_10_Percent 0x06
#define Bias_12_Percent 0x07
#define Bias_14_Percent 0x08
#define Bias_16_Percent 0x09
#define Bias_18_Percent 0x0A
#define Bias_20_Percent 0x0B
#define Bias_22_Percent 0x0C
#define Bias_24_Percent 0x0D

// uncomment if reading temperature from LM91000
// #define TEMP_FLAG

/*                                               */
/* Configure LMP91000 for the intended operation */
/*                                               */
void LMP_MODE(void) {

  /* MODECN register: Configure device mode to Three Lead Amperometric */

  Wire.beginTransmission(LMP91000_Slave_Address);

  /* LOCKN register: Unlock for configuring TIACN and REFCN */
  Wire.write(LOCKCN);
  Wire.write(Write_TIACN_REFCN); // Unlock
  Wire.endTransmission();

  // comment out to see if temp reading are causing problems

  #ifdef TEMP_FLAG
    Wire.beginTransmission(LMP91000_Slave_Address);
    Wire.write(MODECN);
    Wire.write(Temperature_meas_TIA_On); // temperature output mode on
    Wire.endTransmission();
  #else
    Wire.beginTransmission(LMP91000_Slave_Address);
    Wire.write(MODECN);
    Wire.write(Temperature_meas_TIA_Off); // temperature output mode off
    Wire.endTransmission();
//    wdt_reset(); // So the watchdog doesn't trigger
    delay (10);
    Wire.beginTransmission(LMP91000_Slave_Address);
    Wire.write(MODECN);
    Wire.write(Three_Lead_Amperometric); // 3-wire galvanic mode
    Wire.endTransmission();
  #endif

}

/*                                               */
/* Configure LMP91000 for the intended operation */
/*                                               */
void LMP_CFG(void) {

  /* MODECN register: Configure device mode to Three Lead Amperometric*/

  Wire.beginTransmission(LMP91000_Slave_Address);
  /* LOCKN register: Unlock for configuring TIACN and REFCN */
  Wire.write(LOCKCN);
  Wire.write(Write_TIACN_REFCN); // Unlock
  Wire.endTransmission();

  Wire.beginTransmission(LMP91000_Slave_Address);
  Wire.write(MODECN);
  Wire.write(Three_Lead_Amperometric); // 3-wire galvanic mode
  Wire.endTransmission();

  // Sets the Reference Control Register to select the following parameters:
  Wire.beginTransmission(LMP91000_Slave_Address);
  Wire.write(REFCN);
  // for some reason it seems even with bias set to zero percent the positive
  // setting works better than the negative setting, not sure why.
  Wire.write(External_Vref | Internal_Zero_50 | Bias_Negative | Bias_0_Percent);
  Wire.endTransmission();

  /* Sets the TIA Control Register to select 350kohm gain resistance
  * and load resistance (100 ohm)*/
  Wire.beginTransmission(LMP91000_Slave_Address);
  Wire.write(TIACN);
  Wire.write(Gain_External_Resistance | Rload_50_Ohm);
  Wire.endTransmission();
}

#endif //_LMP91000_
