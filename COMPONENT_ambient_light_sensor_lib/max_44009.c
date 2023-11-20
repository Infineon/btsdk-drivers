/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 */

#include "sparcommon.h"
#include "wiced_platform.h"

#include "wiced_hal_i2c.h"
#include "max_44009.h"
#include "wiced_bt_trace.h"

/* Put reg addr into array to make it act as buffer, I2C api need buffer as input param */
uint8_t reg_buffer[3] = {MAX44009_INTERRUPT_STATUS, MAX44009_LUX_HIGH_REGISTER, MAX44009_LUX_LOW_REGISTER};

max44009_reg_info_t max44009_reg_info;

/******************************************************************************
* Function Name: max44009_int_clean
***************************************************************************//**
* Clean interrupt status.
*
* \param  None
*
* \return None
******************************************************************************/
void max44009_int_clean (void)
{
    uint8_t irq_status;
    /* read irq status reg to clear irq status */
    wiced_hal_i2c_write( &reg_buffer[0], 0x0001, MAX44009_ADDRESS1);
    wiced_hal_i2c_read(&irq_status, 0x0001, MAX44009_ADDRESS1);

    WICED_BT_TRACE("max44009 interrupt enters \r\n");
}

/******************************************************************************
* Function Name: max44009_init
***************************************************************************//**
* Initializes the 44009 light sensor.
*
* \param max44009_user_set_t *max44009_usr_set
* Configure user structure.
*
*       uint16_t    scl_pin                     - scl pin definition
*
*       uint16_t    sda_pin                     - sda pin definition
*
*       uint16_t    irq_pin                     - pin used to receive interrupt signal from light sensor
*                                                 Set to WICED_HAL_GPIO_PIN_UNUSED if the IRQ pin is not connected/used
*
*       uint8_t     irq_enable_reg_value        - irq enable register value
*
*       uint8_t     cfg_reg_value               - configuration register value
*
*       uint8_t     upper_threshold_reg_value   - upper threshold register value
*
*       uint8_t     low_threshold_reg_value     - low threshold register value
*
*       uint8_t     threshold_timer_reg_value   - delay time for asserting irq pin when light level exceeds threshold
*
* \param user_fn
* Points to the function to call when the interrupt comes .Below is the description of the arguments received by the cb.
*
*       void* user_data  - User data provided when interrupt is being registered
*                        using wiced_hal_gpio_register_pin_for_interrupt(...)
*
*       uint8_t port_pin - Number of the pin causing the interrupt
*
* \param usr_data
* Will be passed back to user_fn as-is. Typically NULL.
*
* \return None
******************************************************************************/
void max44009_init(max44009_user_set_t *max44009_usr_set, void (*user_fn)(void*, uint8_t), void* usr_data)
{
    WICED_BT_TRACE("max44009_init: SCL:%d SDA:%d\n", max44009_usr_set->scl_pin, max44009_usr_set->sda_pin);

    wiced_hal_i2c_init();
  #if defined(CYW43012C0)
    if(max44009_usr_set->scl_pin < WICED_GPIO_00 && max44009_usr_set->sda_pin < WICED_GPIO_00)
        wiced_hal_i2c_select_pads(max44009_usr_set->scl_pin, max44009_usr_set->sda_pin);
  #elif !defined(CYW55572A1)
    wiced_hal_i2c_select_pads(max44009_usr_set->scl_pin, max44009_usr_set->sda_pin);
  #endif
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    /* Get I2C device, MAX44009 I2C address is 0x4A(74)
     Select configuration register: 0x02 and write 0x40 into it
    0x40 means: Continuous mode, Integration time = 800 ms */
    max44009_reg_info.reg_addr = MAX44009_CFG_REGISTER;
    max44009_reg_info.reg_value = max44009_usr_set->cfg_reg_value;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);

    /* Enable irq */
    max44009_reg_info.reg_addr = MAX44009_INTERRUPT_ENABLE;
    max44009_reg_info.reg_value = max44009_usr_set->irq_enable_reg_value;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);

    /* Set upper threshold */
    max44009_reg_info.reg_addr = MAX44009_UPPER_THRESHOLD;
    max44009_reg_info.reg_value = max44009_usr_set->upper_threshold_reg_value;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);

    /* Set low threshold */
    max44009_reg_info.reg_addr = MAX44009_LOW_THRESHOLD;
    max44009_reg_info.reg_value = max44009_usr_set->low_threshold_reg_value;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);

    /* set dealy time */
    max44009_reg_info.reg_addr = MAX44009_THRESHOLD_TIMER;
    max44009_reg_info.reg_value = max44009_usr_set->threshold_timer_reg_value;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);

    /* Register irq unless it is set as unused */
    if (max44009_usr_set->irq_pin != WICED_HAL_GPIO_PIN_UNUSED)
    {
        wiced_hal_gpio_configure_pin(max44009_usr_set->irq_pin, (GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_LEVEL_LOW), GPIO_PIN_OUTPUT_HIGH);
        wiced_hal_gpio_register_pin_for_interrupt(max44009_usr_set->irq_pin, user_fn, usr_data);
    }
}

/******************************************************************************
* Function Name: max44009_read_ambient_light
***************************************************************************//**
* Read light sensor status.
*
* \param  None
*
* \return lux_value
* Resolution is 0.01, Max range is 167772.14(3octets).
******************************************************************************/
uint32_t max44009_read_ambient_light(void)
{
    uint8_t exponent, mantissa;
    uint8_t rx_data[2] = {0};
    uint32_t temp=0, lux_value=0;

    wiced_hal_i2c_write( &reg_buffer[1], 0x0001, MAX44009_ADDRESS1);
    wiced_hal_i2c_read(  &rx_data[0],    0x0001, MAX44009_ADDRESS1);

    wiced_hal_i2c_write( &reg_buffer[2], 0x0001, MAX44009_ADDRESS1);
    wiced_hal_i2c_read(  &rx_data[1],    0x0001, MAX44009_ADDRESS1);

    /* Convert the data to lumicance */
    exponent = (rx_data[0] & 0xF0) >> 4;
    mantissa = ((rx_data[0] & 0x0F) << 4) | (rx_data[1] & 0x0F);

    /* lux_value = (float)((0x00000001 << exponent) * (mantissa * 0.045)); */
    temp = (0x00000001 << exponent) * mantissa * 45;

    /* merge integer part and decimal part into uint32 */
    lux_value = temp / 10 + (temp / 100 % 10 * 10 + temp / 10 % 10);

   // WICED_BT_TRACE("lux value: %d.%d\r\n", lux_value / 100, lux_value % 100);

    return lux_value;
}

/******************************************************************************
* Function Name: convert_lux_2_reg_value
***************************************************************************//**
* Convert from lux value to a reg value.
*
* \param  lux_value
* lux_value. Range is 0.045 ~ 188000.
*
* \return reg_value
* 8 bits reg value.
******************************************************************************/
uint8_t convert_lux_2_reg_value(uint32_t lux_value)
{
    uint8_t exp, reg_value;
    uint32_t mantissa;

    mantissa = lux_value * 1000 / 45;

    if (mantissa == 0 || mantissa > 4177778)
    {
        /* Error */
        WICED_BT_TRACE("Exceed the lux limitation\r\n");
        return 0;
    }

    for (exp = 0; mantissa > 0xff; exp++)
    {
        mantissa >>= 1;
    }

    mantissa >>= 4;
    mantissa &= 0xf;
    exp <<= 4;

    reg_value = exp | mantissa;
    WICED_BT_TRACE("Threshold register value you set is: %d\r\n", reg_value);

    return reg_value;
}

/******************************************************************************
* Function Name: max44009_set_low_threshold
***************************************************************************//**
* Set low threshold.
*
* \param  lux_value
* Low threshold value. Range is 0.045 ~ 188000.
*
* \return None
******************************************************************************/
void max44009_set_low_threshold(uint32_t lux_value)
{
    uint8_t low_threshold;
    low_threshold = convert_lux_2_reg_value(lux_value);
    max44009_reg_info.reg_addr = MAX44009_LOW_THRESHOLD;
    max44009_reg_info.reg_value = low_threshold;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);
}

/******************************************************************************
* Function Name: max44009_set_upper_threshold
***************************************************************************//**
* Set upper threshold.
*
* \param  lux_value
* Upper threshold value. Range is 0.045 ~ 188000.
*
* \return None
******************************************************************************/
void max44009_set_upper_threshold(uint32_t lux_value)
{
    uint8_t upper_threshold;
    upper_threshold = convert_lux_2_reg_value(lux_value);
    max44009_reg_info.reg_addr = MAX44009_UPPER_THRESHOLD;
    max44009_reg_info.reg_value = upper_threshold;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);
}

/******************************************************************************
* Function Name: max44009_set_irq_enable
***************************************************************************//**
* Set irq enable.
*
* \param  irq_enable
* Irq enable value.
*
* \return None
******************************************************************************/
void max44009_set_irq_enable(uint8_t irq_enable)
{
    max44009_reg_info.reg_addr = MAX44009_INTERRUPT_ENABLE;
    max44009_reg_info.reg_value = irq_enable;
    wiced_hal_i2c_write( (uint8_t*)&max44009_reg_info, 0x0002, MAX44009_ADDRESS1);
}
