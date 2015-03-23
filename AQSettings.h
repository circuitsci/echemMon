#ifndef AQ_SETTINGS_H
#define AQ_SETTINGS_H

// Settings Struct
struct AQSettings {
  uint8_t verification_check;

  uint8_t dataSetId;
  uint16_t readInterval; // Minutes between readings
  uint16_t loadResistor; // AFE Rload setting in ohms, options are 10, 33, 50, 100

  float uutSensitivity; // nA per ppm @ 20degC as measured at Alphasense factory
  float uutOffset;      // nA @ 20degC as measured at Alphasense factory
};

extern AQSettings g_settings;

#endif //AQ_SETTINGS_H
