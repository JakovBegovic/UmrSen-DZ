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
#include "imu-handler.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_INTERRUPT_PRIORITY (7u)

#define GRAVITY_ACCEL 9.8f
#define DEG_TO_RAD_COEF 0.017f // 2*pi/360 = 0,017

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


int16_t transmit_imu[6] = {0};
float *imu_raw_data = (float *)transmit_imu;

/*Data processing*/
float imu_stationary_data[6] = {0};

void change_measurement_units();
void imu_calculate_stationary_values();
void imu_compensate();

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

    imu_calculate_stationary_values();

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

          imu_compensate();

          change_measurement_units();

          /* Display IMU data - assuming the data is organized as [accel_x,
           * accel_y, accel_z, gyro_x, gyro_y, gyro_z] */
          printf("\x1b[2J\x1b[;H \r");
          printf("Accel X     : %0.3f m^2/s\r\n", imu_raw_data[0]);
          printf("Accel Y     : %0.3f m^2/s\r\n", imu_raw_data[1]);
          printf("Accel Z     : %0.3f m^2/s\r\n", imu_raw_data[2]);
          printf("Gyro X      : %0.3f rad/s\r\n", imu_raw_data[3]);
          printf("Gyro Y      : %0.3f rad/s\r\n", imu_raw_data[4]);
          printf("Gyro Z      : %0.3f rad/s\r\n", imu_raw_data[5]);
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

void imu_calculate_stationary_values(){
	// calculate zero offset from arithmetic average
	for(int i=0; i<10; i++){
	    imu_get_data(imu_raw_data);

	    imu_stationary_data[0] += imu_raw_data[0] * 0.1f;
	    imu_stationary_data[1] += imu_raw_data[1] * 0.1f;
	    imu_stationary_data[2] += imu_raw_data[2] * 0.1f;
	    imu_stationary_data[3] += imu_raw_data[3] * 0.1f;
	    imu_stationary_data[4] += imu_raw_data[4] * 0.1f;
	    imu_stationary_data[5] += imu_raw_data[5] * 0.1f;

	    cyhal_system_delay_ms(5);
	}

	imu_stationary_data[2] = imu_raw_data[2] + 2.8f;
}

void imu_compensate(){
	// compensate for zero offset
	imu_raw_data[0] = imu_raw_data[0] - imu_stationary_data[0];
	imu_raw_data[1] = imu_raw_data[1] - imu_stationary_data[1];
	imu_raw_data[2] = imu_raw_data[2] - imu_stationary_data[2];
	imu_raw_data[3] = imu_raw_data[3] - imu_stationary_data[3];
	imu_raw_data[4] = imu_raw_data[4] - imu_stationary_data[4];
	imu_raw_data[5] = imu_raw_data[5] - imu_stationary_data[5];
}

void change_measurement_units(){
	// turn g-unit to m^2/s
	imu_raw_data[0] = imu_raw_data[0] * GRAVITY_ACCEL;
	imu_raw_data[1] = imu_raw_data[1] * GRAVITY_ACCEL;
	imu_raw_data[2] = imu_raw_data[2] * GRAVITY_ACCEL;

	// turn °/s to rad/s
	imu_raw_data[3] = imu_raw_data[3] * DEG_TO_RAD_COEF;
	imu_raw_data[4] = imu_raw_data[4] * DEG_TO_RAD_COEF;
	imu_raw_data[5] = imu_raw_data[5] * DEG_TO_RAD_COEF;
}


/* [] END OF FILE */
