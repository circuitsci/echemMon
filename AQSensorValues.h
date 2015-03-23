#ifndef AQ_SENSOR_VALUES_H
#define AQ_SENSOR_VALUES_H

/* Sensor Values */
struct AQSensorValues {
  float temperature;
  float humidity;

  float echemVdd;
  float echemVin;
  float ppmUUTraw;
  float ppmUUT;
  float uutcomp;
  float volt1diff;
  float volt2diff;
  float volt3;
  float tempDelta;
};

#endif // AQ_SENSOR_VALUES_H
