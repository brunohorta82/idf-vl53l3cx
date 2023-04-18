#pragma once
#ifndef __Vl53lxWrapper
#define __Vl53lxWrapper
#include "I2C.hpp"
#include <vl53lx_class.hpp>
#include "vector"
using namespace std;
using namespace Components;
unsigned long millis();
class Vl53lxSensorReadings
{
private:
  int idx;
  uint8_t status;
  int16_t distance;
  float signalRateRtnMegaCps;
  float ambientRateRtnMegaCps;

public:
  Vl53lxSensorReadings(int idx, uint8_t status,
                       int16_t distance,
                       float signalRateRtnMegaCps,
                       float ambientRateRtnMegaCps)
  {
    this->idx = idx;
    this->status = status;
    this->distance = distance;
    this->signalRateRtnMegaCps = signalRateRtnMegaCps;
    this->ambientRateRtnMegaCps = ambientRateRtnMegaCps;
  }
  int getIdx()
  {
    return idx;
  }
  uint8_t getStatus()
  {
    return status;
  }
  int16_t getDistance()
  {
    return distance;
  }
  float getSignalRateRtnMegaCps()
  {
    return signalRateRtnMegaCps;
  }
  float getAmbientRateRtnMegaCps()
  {
    return ambientRateRtnMegaCps;
  }
};
class Vl53lxSensor
{
private:
  unsigned long timeSinceLastInterruptClear = millis();
  std::shared_ptr<I2CMaster> i2c;
  VL53LX vl53lxInstance;

public:
  Vl53lxSensor(
      int xshutPin,
      int interruptPin,
      int sensorDeviceAddress,
      std::shared_ptr<I2CMaster> i2cBus);
  vector<Vl53lxSensorReadings> getLatestMeasurement();
};
#endif