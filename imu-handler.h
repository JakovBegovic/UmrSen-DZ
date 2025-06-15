/*
 * imu-handler.h
 *
 *  Created on: Jun 15, 2025
 *      Author: jakovbegovic
 */

#ifndef IMU_HANDLER_H_
#define IMU_HANDLER_H_

#include "mtb_bmi160.h"
#include "cyhal.h"


#define IMU_SPI_FREQUENCY 10000000
#define IMU_SCAN_RATE 50
#define IMU_TIMER_FREQUENCY 100000
#define IMU_TIMER_PERIOD (IMU_TIMER_FREQUENCY / IMU_SCAN_RATE)
#define IMU_TIMER_PRIORITY 3


volatile bool imu_flag = false;

/* BMI160 driver structures */
mtb_bmi160_data_t data;
mtb_bmi160_t sensor_bmi160;
/* SPI object for data transmission */
cyhal_spi_t spi;
/* Global timer used for getting data */
cyhal_timer_t imu_timer;


void imu_interrupt_handler(void *callback_arg, cyhal_timer_event_t event) {
  (void)callback_arg;
  (void)event;

  imu_flag = true;
}

cy_rslt_t imu_timer_init(void) {
  cy_rslt_t rslt;
  const cyhal_timer_cfg_t timer_cfg = {
      .compare_value = 0,              /* Timer compare value, not used */
      .period = IMU_TIMER_PERIOD,      /* Defines the timer period */
      .direction = CYHAL_TIMER_DIR_UP, /* Timer counts up */
      .is_compare = false,             /* Don't use compare mode */
      .is_continuous = true,           /* Run the timer indefinitely */
      .value = 0                       /* Initial value of counter */
  };

  /* Initialize the timer object. Does not use pin output ('pin' is NC) and
   * does not use a pre-configured clock source ('clk' is NULL). */
  rslt = cyhal_timer_init(&imu_timer, NC, NULL);
  if (CY_RSLT_SUCCESS != rslt) {
    return rslt;
  }

  /* Apply timer configuration such as period, count direction, run mode, etc.
   */
  rslt = cyhal_timer_configure(&imu_timer, &timer_cfg);
  if (CY_RSLT_SUCCESS != rslt) {
    return rslt;
  }

  /* Set the frequency of timer to 100KHz */
  rslt = cyhal_timer_set_frequency(&imu_timer, IMU_TIMER_FREQUENCY);
  if (CY_RSLT_SUCCESS != rslt) {
    return rslt;
  }

  /* Assign the ISR to execute on timer interrupt */
  cyhal_timer_register_callback(&imu_timer, imu_interrupt_handler, NULL);
  /* Set the event on which timer interrupt occurs and enable it */
  cyhal_timer_enable_event(&imu_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                           IMU_TIMER_PRIORITY, true);
  /* Start the timer with the configured settings */
  rslt = cyhal_timer_start(&imu_timer);
  if (CY_RSLT_SUCCESS != rslt) {
    return rslt;
  }

  return CY_RSLT_SUCCESS;
}


cy_rslt_t imu_init(void) {
	cy_rslt_t result;

	/* Initialize SPI for IMU communication */
	result = cyhal_spi_init(&spi, CYBSP_SPI_MOSI, CYBSP_SPI_MISO, CYBSP_SPI_CLK,
						  NC, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);
	if (CY_RSLT_SUCCESS != result) {
	return result;
	}

	/* Set SPI frequency to 10MHz */
	result = cyhal_spi_set_frequency(&spi, IMU_SPI_FREQUENCY);
	if (CY_RSLT_SUCCESS != result) {
	return result;
	}

	/* Initialize the chip select line */
	result = cyhal_gpio_init(CYBSP_SPI_CS, CYHAL_GPIO_DIR_OUTPUT,
						   CYHAL_GPIO_DRIVE_STRONG, 1);
	if (CY_RSLT_SUCCESS != result) {
	return result;
	}

	/* Initialize the IMU */
	result = mtb_bmi160_init_spi(&sensor_bmi160, &spi, CYBSP_SPI_CS);
	if (CY_RSLT_SUCCESS != result) {
	return result;
	}

	/* Set the output data rate and range of the accelerometer */
	/* Accelerometer */
	sensor_bmi160.sensor.accel_cfg.odr =
	  BMI160_ACCEL_ODR_50HZ; // 50 Hz sampling rate
	sensor_bmi160.sensor.accel_cfg.range = BMI160_ACCEL_RANGE_8G; // ±8g range

	/* Gyroscope */
	sensor_bmi160.sensor.gyro_cfg.odr =
	  BMI160_GYRO_ODR_50HZ; // same 50 Hz sampling rate
	sensor_bmi160.sensor.gyro_cfg.range =
	  BMI160_GYRO_RANGE_2000_DPS; // ±2000 degrees per second

	/* Set the sensor configuration */
	bmi160_set_sens_conf(&(sensor_bmi160.sensor));

	imu_flag = false;

	/* Timer for data collection */
	result = imu_timer_init();
	if (CY_RSLT_SUCCESS != result) {
	return result;
	}

	return CY_RSLT_SUCCESS;
}

void imu_get_data(float *imu_data) {
	cy_rslt_t result;
	result = mtb_bmi160_read(&sensor_bmi160, &data);
	if (CY_RSLT_SUCCESS != result) {
	CY_ASSERT(0);
	}

	imu_data[0] = ((float)data.accel.x) / (float)0x1000;
	imu_data[1] = ((float)data.accel.y) / (float)0x1000;
	imu_data[2] = ((float)data.accel.z) / (float)0x1000;
	imu_data[3] = ((float)data.gyro.x) / 16.4f;
	imu_data[4] = ((float)data.gyro.y) / 16.4f;
	imu_data[5] = ((float)data.gyro.z) / 16.4f;
}


#endif /* IMU_HANDLER_H_ */
