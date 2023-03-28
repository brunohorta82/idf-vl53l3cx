#pragma once
#ifndef __Vl53lxWrapper
#define __Vl53lxWrapper
#include "I2C.hpp"
#include <vl53lx_class.hpp>
using namespace Components;
unsigned long millis();
class Vl53lxWrapper
{
private:
  unsigned long timeSinceLastInterruptClear = millis();

  std::shared_ptr<I2CMaster> i2c;
  VL53LX vl53lxInstance;
  VL53LX_MultiRangingData_t multiRangingData;

  void endlessLoop();

  void clearInterrupt();

public:
  Vl53lxWrapper(
      int xshutPin,
      int interruptPin,
      int sensorDeviceAddress,
      std::shared_ptr<I2CMaster> i2cBus);

  VL53LX_MultiRangingData_t getLatestMeasurement();

  ~Vl53lxWrapper();
};
#endif