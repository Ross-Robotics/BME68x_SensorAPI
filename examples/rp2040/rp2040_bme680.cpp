/* ===================================================================
 * © Copyright 2025 - Ross Robotics. All rights reserved.
 * Unauthorised duplicating, while flattering, is strictly prohibited.
 * Our robots are watching. 🤖
 * 
 * Device: Bosch BME68X Sensor
 * Software Version: 1.0
 * File: src/main.cpp
 * ===================================================================
 */

#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include "communications_implementation.hpp"
#include "hardware_definitions.hpp"

extern "C" {
#include "bme68x.h"
}

constexpr size_t I2C_FREQUENCY = 100 * 1000;

BME68X_INTF_RET_TYPE bme68x_i2c_write(
  uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t device_addr = *(uint8_t *)intf_ptr;

  (void)intf_ptr;

  return i2c_write(device_addr, reg_addr, reg_data, len) ? BME68X_OK : BME68X_E_COM_FAIL;
}

BME68X_INTF_RET_TYPE bme68x_i2c_read(
  uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t device_addr = *(uint8_t *)intf_ptr;

  (void)intf_ptr;

  return i2c_read(device_addr, reg_addr, reg_data, len) ? BME68X_OK : BME68X_E_COM_FAIL;
}

void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
  (void)intf_ptr;
  sleep_us(period);
}

int main()
{
  stdio_init_all();
  sleep_ms(2000);

  // Initialize I2C
  i2c_init(i2c0, I2C_FREQUENCY);
  gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C0_SDA_PIN);
  gpio_pull_up(I2C0_SCL_PIN);

  bme68x_dev bme68x;
  bme68x.write = bme68x_i2c_write;
  bme68x.read = bme68x_i2c_read;
  bme68x.intf = BME68X_I2C_INTF;
  bme68x.intf_ptr = const_cast<void *>(static_cast<const void *>(&GAS_SENSOR_I2C_ADDR));
  bme68x.delay_us = bme68x_delay_us;
  bme68x.amb_temp = 25;

  if (bme68x_init(&bme68x) != BME68X_OK) {
    printf("Failed to initialize BME68X sensor\n");
  } else {
    printf("BME68X sensor initialized\n");
  }

  struct bme68x_conf conf;
  struct bme68x_heatr_conf heatr_conf;
  struct bme68x_data data;

  /* Check if rslt == BME68X_OK, report or handle if otherwise */
  conf.filter = BME68X_FILTER_SIZE_127;
  conf.odr = BME68X_ODR_NONE;
  conf.os_hum = BME68X_OS_16X;
  conf.os_pres = BME68X_OS_1X;
  conf.os_temp = BME68X_OS_2X;
  bme68x_set_conf(&conf, &bme68x);

  /* Check if rslt == BME68X_OK, report or handle if otherwise */
  heatr_conf.enable = BME68X_ENABLE;
  heatr_conf.heatr_temp = 300;
  heatr_conf.heatr_dur = 100;
  bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme68x);

  // Main loop state variables
  bool measurement_started = false;
  absolute_time_t meas_start_time = {0};
  uint32_t meas_dur_us = 0;
  uint8_t n_data = 0;

  while (true) {
    if (!measurement_started) {
      meas_dur_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme68x);
      int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x);
      if (rslt != BME68X_OK) {
        printf("Failed to start measurement (code %d)\n", rslt);
      } else {
        meas_start_time = get_absolute_time();
        measurement_started = true;
      }
    } else {
      if (absolute_time_diff_us(meas_start_time, get_absolute_time()) >= meas_dur_us) {
        int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_data, &bme68x);
        if (rslt == BME68X_OK) {
          printf(
            "Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%, Gas resistance: %.2f\n",
            data.temperature, data.pressure, data.humidity, data.gas_resistance);
        } else {
          printf("Failed to get sensor data (code %d)\n", rslt);
        }
        measurement_started = false;
      }
    }
  }
}
