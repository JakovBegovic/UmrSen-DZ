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
#include "xensiv_dps3xx_mtb.h"
#include "xensiv_dps3xx.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define OVERSAMPLING            7
#define I2C_MASTER_FREQUENCY    400000

#define GPIO_INTERRUPT_PRIORITY (7u)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Context for dps310 */
xensiv_dps3xx_t dps310_sensor;

/* Declaration for i2c handler */
cyhal_i2c_t I2Cm_HW;

/* Define the I2C master configuration structure */
cyhal_i2c_cfg_t i2c_cfg_master = {
        CYHAL_I2C_MODE_MASTER,
        0,                          /* address is not used for master mode */
        I2C_MASTER_FREQUENCY
};

/* Context for interrupts */
/* Semaphore from interrupt handler to background process */
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

/*Timer support*/
cyhal_timer_t timer;

void init_timer(void);
uint32_t get_ticks(void);

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
    uint32_t revisionID = 0;
    float pressure, temperature;

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

    /* Initialize i2c for pressure sensor */
    result = cyhal_i2c_init(&I2Cm_HW, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nI2C initialization failed\r\n");
        CY_ASSERT(0);
    }

    /* Configure i2c with master configurations */
    result = cyhal_i2c_configure(&I2Cm_HW, &i2c_cfg_master);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nFailed to configure I2C\r\n");
        CY_ASSERT(0);
    }

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&dps310_sensor, &I2Cm_HW,
                                        XENSIV_DPS3XX_I2C_ADDR_DEFAULT);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nFailed to initialize DPS310 I2C\r\n");
        CY_ASSERT(0);
    }

    /* Retrieve the DPS310 Revision ID and display the same */
    if (xensiv_dps3xx_get_revision_id(&dps310_sensor,(uint8_t*)&revisionID) == CY_RSLT_SUCCESS)
    {
        printf("DPS310 Revision ID = %d\r\n\n",(uint8_t)revisionID);
    }
    else
    {
        printf("Failed to get Revision ID\r\n");
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

    for (;;)
    {
        /* Check the interrupt status */
        if (true == gpio_intr_flag)
        {
            /* Reset interrupt flag */
            gpio_intr_flag = false;
            /* Read the pressure and temperature data */
            if (xensiv_dps3xx_read(&dps310_sensor, &pressure, &temperature) == CY_RSLT_SUCCESS)
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
    }
}


/*******************************************************************************
* Function Name: init_timer
********************************************************************************
* Summary:
*   Initializes a timer to tick every 1 microsecond.
*
* Parameters:
*  void
*
*******************************************************************************/
void init_timer(void) {
    cyhal_timer_cfg_t timer_cfg = {
        .compare_value = 0,
        .period = 0xFFFFFFFF, // Max period for continuous running
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = false,
        .is_continuous = true,
        .value = 0
    };
    cyhal_timer_init(&timer, NC, NULL); // NC = no connect pin
    cyhal_timer_configure(&timer, &timer_cfg);
    cyhal_timer_set_frequency(&timer, 1000000); // 1 MHz for microsecond resolution
    cyhal_timer_start(&timer);
}


/*******************************************************************************
* Function Name: get_ticks
********************************************************************************
* Summary:
*   Returns current timer count in microseconds.
*
* Parameters:
*  void
*
*******************************************************************************/
uint32_t get_ticks(void) {
    return cyhal_timer_read(&timer);
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

	if (current_time - last_time > 500000) { // 500000 microseconds = 500 miliseconds
		gpio_intr_flag = true;

		last_time = current_time;
	}

}

/* [] END OF FILE */
