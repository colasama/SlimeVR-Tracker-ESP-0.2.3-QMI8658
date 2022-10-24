#ifndef QMC5883L_H
#define QMC5883L_H

#include "I2Cdev.h"
#include <Arduino.h>

class QMC5883L {
public:
  void initialize();
  void reset();
  int  ready();
  void reconfig();
  
  int readHeading();

  // Raw mag data
  void getMagetometerData( int16_t *mx, int16_t *my, int16_t *mz);

  void resetCalibration();

  void setSamplingRate( int rate );
  void setRange( int range );
  void setOversampling( int ovl );
  
private:
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;
};

#endif
