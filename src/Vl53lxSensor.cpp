
#include <vl53lx_class.hpp>
#include "Vl53lxSensor.hpp"
#include "esp_timer.h"
#include "interrupt.hpp"
#include <esp_log.h>
#include "driver/gpio.h"
#ifdef CONFIG_VL53L3CX_ACTIVE
Vl53lxSensor::Vl53lxSensor(
    int xshutPin,
    int interruptPin,
    int sensorDeviceAddress,
    std::shared_ptr<I2CMaster> i2cBus) : i2c(i2cBus),
                                         vl53lxInstance(i2c, xshutPin)
{
  VL53LX_Error error = VL53LX_ERROR_NONE;
  vl53lxInstance.begin();

  if (interruptPin != GPIO_NUM_NC)
  {
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << interruptPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)interruptPin, intaISR, (void *)interruptPin);
  }
  if (xshutPin != GPIO_NUM_NC)
  {
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << xshutPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
  }
  vl53lxInstance.VL53LX_Off();
  error = vl53lxInstance.InitSensor(sensorDeviceAddress);
  if (ESP_OK == error)
    ESP_LOGE("VL53LX", "ERROR: %d", error);
  vl53lxInstance.VL53LX_ClearInterruptAndStartMeasurement();
  interruptTriggered = false;
}

vector<Vl53lxSensorReadings> Vl53lxSensor::getLatestMeasurement()
{
  vector<Vl53lxSensorReadings> readings;
  VL53LX_MultiRangingData_t MultiRangingData;
  VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;

  if (interruptTriggered || CONFIG_VL53_INTERRUPT_PIN == GPIO_NUM_NC)
  {
    int status;
    if (CONFIG_VL53_INTERRUPT_PIN == GPIO_NUM_NC)
    {
      do
      {
        status = vl53lxInstance.VL53LX_GetMeasurementDataReady(&NewDataReady);
      } while (!NewDataReady);
    }
    else
    {
      interruptTriggered = false;
      status = vl53lxInstance.VL53LX_GetMeasurementDataReady(&NewDataReady);
    }

    if ((!status) && (NewDataReady != 0))
    {
      status = vl53lxInstance.VL53LX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
      for (j = 0; j < no_of_object_found; j++)
      {
        if (pMultiRangingData->RangeData[j].RangeStatus != 0)
          continue;
        readings.push_back(Vl53lxSensorReadings{j, pMultiRangingData->RangeData[j].RangeStatus, pMultiRangingData->RangeData[j].RangeMilliMeter, (float)(pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0), (float)(pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0)});
        ESP_LOGI("VL", "status=%d Distance=%dmm Signal=%f Ambient%fmcps", pMultiRangingData->RangeData[j].RangeStatus, pMultiRangingData->RangeData[j].RangeMilliMeter, (float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0, (float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
      }
      if (status == 0)
      {
        status = vl53lxInstance.VL53LX_ClearInterruptAndStartMeasurement();
      }
    }
  }

  return readings;
}
#endif