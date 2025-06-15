/*
 * timer-handler.h
 *
 *  Created on: Jun 15, 2025
 *      Author: jakovbegovic
 */

#ifndef TIMER_HANDLER_H_
#define TIMER_HANDLER_H_

#include "cyhal_timer.h"

/*Timer support*/
cyhal_timer_t timer;

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

#endif /* TIMER_HANDLER_H_ */
