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
  uint8_t status;
  int16_t distance;
  FixPoint1616_t signalRateRtnMegaCps;
  FixPoint1616_t ambientRateRtnMegaCps;
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