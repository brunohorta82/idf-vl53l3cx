
#include <vl53lx_class.hpp>
#include "Vl53lxSensor.hpp"
#include "interrupt.hpp"
#include "esp_timer.h"
#include <esp_log.h>
#include "driver/gpio.h"
Vl53lxWrapper::Vl53lxWrapper(
    int xshutPin,
    int interruptPin,
    int sensorDeviceAddress,
    std::shared_ptr<I2CMaster> i2cBus) : i2c(i2cBus),
                                         vl53lxInstance(i2c, xshutPin)
{
  VL53LX_Error error = VL53LX_ERROR_NONE;

  vl53lxInstance.begin();
  vl53lxInstance.VL53LX_Off();
  error = vl53lxInstance.InitSensor(sensorDeviceAddress);
  gpio_install_isr_service(0);
  gpio_set_direction((gpio_num_t)interruptPin, GPIO_MODE_INPUT);
  gpio_pulldown_en((gpio_num_t)interruptPin);
  gpio_pullup_dis((gpio_num_t)interruptPin);
  gpio_set_intr_type((gpio_num_t)interruptPin, GPIO_INTR_NEGEDGE);
  gpio_isr_handler_add((gpio_num_t)interruptPin, intaISR, (void *)interruptPin);
  ESP_LOGI("VL53LX", "ERROR: %d", error);
  // TODO pinMode(interruptPin, INPUT_PULLUP);
  // TODO  attachInterrupt(digitalPinToInterrupt(interruptPin), intaISR, FALLING);
  clearInterrupt();
}

void Vl53lxWrapper::endlessLoop()
{
  while (true)
  {
    delay(100);
  }
}

void Vl53lxWrapper::clearInterrupt()
{
  interruptTriggered = false;
  vl53lxInstance.VL53LX_ClearInterruptAndStartMeasurement();
  timeSinceLastInterruptClear = millis();
}

VL53LX_MultiRangingData_t Vl53lxWrapper::getLatestMeasurement()
{
  int status = 0;

  if (!interruptTriggered)
  {
    if ((millis() - timeSinceLastInterruptClear) > 1000)
    {
      clearInterrupt();
    }
    return multiRangingData;
  }

  uint8_t NewDataReady = 0;
  status = vl53lxInstance.VL53LX_GetMeasurementDataReady(&NewDataReady);
  if (status != 0 || NewDataReady == 0)
    return multiRangingData;

  status = vl53lxInstance.VL53LX_GetMultiRangingData(&multiRangingData);
  clearInterrupt();
  if (status != 0)
    return multiRangingData;

  return multiRangingData;
}

Vl53lxWrapper::~Vl53lxWrapper()
{
  // TODO i2c.~TwoWire();
}
