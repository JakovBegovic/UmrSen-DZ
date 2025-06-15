/*
 * i2c-handler.h
 *
 *  Created on: Jun 15, 2025
 *      Author: jakovbegovic
 */

#ifndef I2C_HANDLER_H_
#define I2C_HANDLER_H_

#define OVERSAMPLING            7
#define I2C_MASTER_FREQUENCY    400000

/* Declaration for i2c handler */
cyhal_i2c_t I2Cm_HW;

/* Define the I2C master configuration structure */
cyhal_i2c_cfg_t i2c_cfg_master = {
        CYHAL_I2C_MODE_MASTER,
        0,                          /* address is not used for master mode */
        I2C_MASTER_FREQUENCY
};

cy_rslt_t init_i2c(){
	/* Initialize i2c for pressure sensor */
	cy_rslt_t result = cyhal_i2c_init(&I2Cm_HW, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);

	if (result != CY_RSLT_SUCCESS)
	{
	    printf("\r\nI2C initialization failed\r\n");
	    CY_ASSERT(0);
	}

	return result;
}

cy_rslt_t configure_i2c(){
    /* Configure i2c with master configurations */
	cy_rslt_t result = cyhal_i2c_configure(&I2Cm_HW, &i2c_cfg_master);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nFailed to configure I2C\r\n");
        CY_ASSERT(0);
    }

    return result;
}

void setup_i2c(){
	cy_rslt_t result;
	result = init_i2c() && configure_i2c();
}




#endif /* I2C_HANDLER_H_ */
