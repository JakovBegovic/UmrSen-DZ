/*
 * temp-init.h
 *
 *  Created on: Jun 15, 2025
 *      Author: jakovbegovic
 */

#ifndef TEMP_READER_H_
#define TEMP_READER_H_

#include "xensiv_dps3xx_mtb.h"
#include "xensiv_dps3xx.h"

/* Context for dps310 */
xensiv_dps3xx_t dps310_sensor;

cy_rslt_t init_dsp310(cyhal_i2c_t* I2Cm_HW){
    /* Initialize pressure sensor */
    cy_rslt_t init_result = xensiv_dps3xx_mtb_init_i2c(&dps310_sensor, I2Cm_HW,
                                        XENSIV_DPS3XX_I2C_ADDR_DEFAULT);
    if (init_result != CY_RSLT_SUCCESS)
    {
        printf("\r\nFailed to initialize DPS310 I2C\r\n");
        CY_ASSERT(0);
    }

    /* Retrieve the DPS310 Revision ID and display the same */
    uint32_t revisionID = 0;

    if (xensiv_dps3xx_get_revision_id(&dps310_sensor,(uint8_t*)&revisionID) == CY_RSLT_SUCCESS)
    {
        printf("DPS310 Revision ID = %d\r\n\n",(uint8_t)revisionID);
    }
    else
    {
        printf("Failed to get Revision ID\r\n");
        CY_ASSERT(0);
    }

    return init_result;
}



#endif /* TEMP_READER_H_ */
