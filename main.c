/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the DPS310 pressure sensor Application
*
* Related Document: See Readme.md
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_bmi160.h"

#include "temp-reader.h"
#include "timer-handler.h"
#include "i2c-handler.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_INTERRUPT_PRIORITY (7u)

#define IMU_SPI_FREQUENCY 10000000
#define IMU_SCAN_RATE 50
#define IMU_TIMER_FREQUENCY 100000
#define IMU_TIMER_PERIOD (IMU_TIMER_FREQUENCY / IMU_SCAN_RATE)
#define IMU_TIMER_PRIORITY 3

/*******************************************************************************
* Global Variables
********************************************************************************/

/* Context for interrupts */
volatile bool gpio_intr_flag = false;

volatile uint32_t last_time = 0;
volatile uint32_t current_time = 0;

static void gpio_interrupt_handler_HAL(void *arg, cyhal_gpio_event_t event);

/*This structure is used to initialize callback*/
cyhal_gpio_callback_data_t cb_data =
    {
        .callback = gpio_interrupt_handler_HAL,
        .callback_arg = NULL
 };


/* BMI160 driver structures */
mtb_bmi160_data_t data;
mtb_bmi160_t sensor_bmi160;
/* SPI object for data transmission */
cyhal_spi_t spi;
/* Global timer used for getting data */
cyhal_timer_t imu_timer;

int16_t transmit_imu[6] = {0};
float *imu_raw_data = (float *)transmit_imu;

volatile bool imu_flag = false;

void imu_interrupt_handler(void *callback_arg, cyhal_timer_event_t event);
cy_rslt_t imu_init(void);
cy_rslt_t imu_timer_init(void);
void imu_get_data(float *imu_data);

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function for CM4 CPU. It perfroms the following opeartions:
 *    1. Initializes the BSP
 *    2. Initializes retarget IO for UART debug printing
 *    3. Initializes I2C using HAL driver
 *    4. Initializes the DPS310 pressure sensor
 *    5. Initializes the timer
 *    6. Initializes the user button
 *    7. Configures user button interrupts
 *    8. Measures the temperature and the pressure values and prints it on the
*        serial terminal every time the interrupt is triggerd if the time elapsed from the last interrupt is more than 500 ms.
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H \r");
    printf("=========================================================\n\r");
    printf("  PSoC 6 MCU:  Interfacing DPS310 Pressure Sensor \r\n");
    printf("=========================================================\n\n\r");

    setup_i2c();

    result = init_dsp310(&I2Cm_HW);


    /* Initialize IMU sensor */
    result = imu_init();
    if (result != CY_RSLT_SUCCESS) {
      printf("\r\nFailed to initialize IMU sensor\r\n");
      CY_ASSERT(0);
    }


    /*Initialize the timer*/
    init_timer();

    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                                CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
	if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_RISE,
                            GPIO_INTERRUPT_PRIORITY, true);
    
    printf("Setup izvrsen\r\n\r\n");

    float pressure, temperature;
    for (;;)
    {
        /* Check the interrupt status */
        if (true == gpio_intr_flag)
        {
            /* Reset interrupt flag */
            gpio_intr_flag = false;
            /* Read the pressure and temperature data */
            if (read_temp(&pressure, &temperature) == CY_RSLT_SUCCESS)
            {
                /* Display the pressure and temperature data in console*/
                printf("Pressure : %0.2f mBar", pressure);
                /* 0xF8 - ASCII Degree Symbol */
                printf("\t Temperature: %0.2f %cC \r\n", temperature, 0xF8);
            }
            else
            {
                printf("\n Failed to read temperature and pressure data.\r\n");
                CY_ASSERT(0);
            }
    	}


        if (imu_flag == true) {
          imu_flag = false;
          imu_get_data(imu_raw_data);

          /* Display IMU data - assuming the data is organized as [accel_x,
           * accel_y, accel_z, gyro_x, gyro_y, gyro_z] */
          printf("Accel X     : %0.3f g\r\n", imu_raw_data[0]);
          printf("Accel Y     : %0.3f g\r\n", imu_raw_data[1]);
          printf("Accel Z     : %0.3f g\r\n", imu_raw_data[2]);
          printf("Gyro X      : %0.3f dps\r\n", imu_raw_data[3]);
          printf("Gyro Y      : %0.3f dps\r\n", imu_raw_data[4]);
          printf("Gyro Z      : %0.3f dps\r\n", imu_raw_data[5]);
        }
    }
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler_HAL
********************************************************************************
* Summary:
*   GPIO interrupt handler for user button. Implements software debouncing.
*   The debounce time is 500 ms, which has shown to be optimal for reading data
*   from this sensor.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
*******************************************************************************/
static void gpio_interrupt_handler_HAL(void *arg, cyhal_gpio_event_t event)
{
	current_time = get_ticks();

	if (current_time - last_time > 1000000) { // 1000000 microseconds = 1000 miliseconds
		gpio_intr_flag = true;
		imu_flag = true;

		last_time = current_time;
	}

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

void imu_interrupt_handler(void *callback_arg, cyhal_timer_event_t event) {
  (void)callback_arg;
  (void)event;

  imu_flag = true;
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
/* [] END OF FILE */
