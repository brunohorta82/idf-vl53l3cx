
#include <vl53lx_class.hpp>
#include "Vl53lxSensor.hpp"
#include "interrupt.hpp"
#include "esp_timer.h"
#include <esp_log.h>
#include "driver/gpio.h"
Vl53lxSensor::Vl53lxSensor(
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
  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask = (1ULL << interruptPin);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)interruptPin, intaISR, (void *)interruptPin);
  if (ESP_OK == error)
    ESP_LOGE("VL53LX", "ERROR: %d", error);
  clearInterrupt();
}

void Vl53lxSensor::endlessLoop()
{
  while (true)
  {
    delay(100);
  }
}

void Vl53lxSensor::clearInterrupt()
{
  interruptTriggered = false;
  vl53lxInstance.VL53LX_ClearInterruptAndStartMeasurement();
  timeSinceLastInterruptClear = millis();
}

VL53LX_MultiRangingData_t Vl53lxSensor::getLatestMeasurement()
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

Vl53lxSensor::~Vl53lxSensor()
{
  // TODO i2c.~TwoWire();
}
