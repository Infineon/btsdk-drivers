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
#include "opt3002.h"
#include "wiced_bt_trace.h"

/* Put reg addr into array to make it act as buffer, I2C api need buffer as input param */
uint8_t reg_buffer[1] = {OPT3002_RESULT_REGISTER};

opt3002_reg_info_t opt3002_reg_info;

/******************************************************************************
* Function Name: opt3002_int_clean
***************************************************************************//**
* Clean interrupt status.
*
* \param  None
*
* \return None
******************************************************************************/
void opt3002_int_clean (void)
{
    uint8_t irq_status;
    /* read irq status reg to clear irq status */
    wiced_hal_i2c_write( &reg_buffer[0], 0x0001, OPT3002_ADDRESS1);
    wiced_hal_i2c_read(&irq_status, 0x0001, OPT3002_ADDRESS1);

    WICED_BT_TRACE("opt3002 interrupt enters \r\n");
}

/******************************************************************************
* Function Name: opt3002_register_read
***************************************************************************//**
* read a 16-bit register.
*
* \param  OPT3002_REG_t register value
*
* \return 16-bit register value
******************************************************************************/
uint16_t opt3002_register_read (OPT3002_REG_t reg)
{
    opt3002_reg_info.reg_addr = reg;
    wiced_hal_i2c_combined_read((uint8_t*)&opt3002_reg_info.reg_value_hi, 2,
                                (uint8_t*)&opt3002_reg_info.reg_addr, 1, OPT3002_ADDRESS1);
    return ((opt3002_reg_info.reg_value_hi << 8) | opt3002_reg_info.reg_value_lo);
}

/******************************************************************************
* Function Name: opt3002_register_write
***************************************************************************//**
* read a 16-bit register.
*
* \param  OPT3002_REG_t register id
* \param  uint16_t      register value
*
* \return status I2CM_SUCCESS or I2CM_OP_FAILED
******************************************************************************/
uint8_t opt3002_register_write (OPT3002_REG_t reg, uint16_t value)
{
    opt3002_reg_info.reg_addr = reg;
    opt3002_reg_info.reg_value_hi = value >> 8;
    opt3002_reg_info.reg_value_lo = value & 0xff;
    return wiced_hal_i2c_write( (uint8_t*)&opt3002_reg_info, 0x0003, OPT3002_ADDRESS1);
}

/******************************************************************************
* Function Name: opt3002_init
***************************************************************************//**
* Initializes the opt3002 light sensor.
*
* \param opt3002_user_set_t *opt3002_usr_set
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
void opt3002_init(opt3002_user_set_t *opt3002_usr_set, void (*user_fn)(void*, uint8_t), void* usr_data)
{
    uint16_t reg_value;
    WICED_BT_TRACE("opt3002_init: SCL:%d SDA:%d\n", opt3002_usr_set->scl_pin, opt3002_usr_set->sda_pin);

    wiced_hal_i2c_init();
  #if defined(CYW43012C0)
    if(opt3002_usr_set->scl_pin < WICED_GPIO_00 && opt3002_usr_set->sda_pin < WICED_GPIO_00)
        wiced_hal_i2c_select_pads(opt3002_usr_set->scl_pin, opt3002_usr_set->sda_pin);
  #elif !defined(CYW55572A1)
    wiced_hal_i2c_select_pads(opt3002_usr_set->scl_pin, opt3002_usr_set->sda_pin);
  #endif
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    /* read manufacturer ID */
    reg_value = opt3002_register_read(OPT3002_MANUFACTURER_ID);
    WICED_BT_TRACE("Read manufacturer's ID register 0x7E: %c%c\n", (uint8_t)(reg_value >> 8), (uint8_t)(reg_value & 0xff));

    /* config I2C device, opt3002 I2C address is 0x44
     Select configuration register: 0x01 and write 0xC600 */
    opt3002_register_write(OPT3002_CFG_REGISTER, OPT3002_CONFIG_DEFAULT);

    /* Set upper threshold */
    opt3002_register_write(OPT3002_UPPER_THRESHOLD, 0xbfff);

    /* Set low threshold */
    opt3002_register_write(OPT3002_LOW_THRESHOLD, 0);

    /* Register irq unless it is set as unused */
    if (opt3002_usr_set->irq_pin != WICED_HAL_GPIO_PIN_UNUSED)
    {
        wiced_hal_gpio_configure_pin(opt3002_usr_set->irq_pin, (GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_LEVEL_LOW), GPIO_PIN_OUTPUT_HIGH);
        wiced_hal_gpio_register_pin_for_interrupt(opt3002_usr_set->irq_pin, user_fn, usr_data);
    }
}

/******************************************************************************
* Function Name: opt3002_read_ambient_light
***************************************************************************//**
* Read light sensor status.
*
* \param  None
*
* \return optical_power
* Resolution is 0.01, Max range is 167772.14(3octets).
******************************************************************************/
uint32_t opt3002_read_ambient_light(void)
{
    uint16_t exponent, mantissa;
    uint32_t temp=0, optical_power=0;
    uint16_t reg_value;

    reg_value = opt3002_register_read(OPT3002_RESULT_REGISTER);

    /* Convert the data to irradiance nW/m^2 */
    exponent = reg_value >> 12;
    mantissa = reg_value & 0xfff;

    /* lux_value = (float)((0x00000001 << exponent) * (mantissa * 0.045)); */
    /* Optical_Power = (2^[3:0]) × R[11:0] × 1.2 [nW/cm2]  */
    temp = (0x00000001 << exponent) * mantissa * 120;

    /* merge integer part and decimal part into uint32 */
    optical_power = temp / 10 + (temp / 100 % 10 * 10 + temp / 10 % 10);

  //  WICED_BT_TRACE("optical power value: %d.%d nW/cm^2\r\n", optical_power / 100, optical_power % 100);

    return optical_power;
}

/******************************************************************************
* Function Name: convert_lux_2_reg_value
***************************************************************************//**
* Convert from lux value to a reg value.
*
* \param  optical_power
* optical_power. Range is 1.2 nW/cm2 to 10,000,000 nW/cm2
*
* \return reg_value 16-bit register value.
******************************************************************************/
uint16_t convert_lux_2_reg_value(uint32_t optical_power)
{
    uint16_t exp;
    uint16_t reg_value = 0xffff;
    uint32_t mantissa;

    mantissa = optical_power * 100 / 120;

    for (exp = 0; mantissa > 0xfff; exp++)
    {
        mantissa >>= 1;
    }
    if(exp > 0xd)
    {
        WICED_BT_TRACE("optical power value exceeds sensor limits\n;");
    }
    else
    {
        mantissa &= 0xfff;
        exp <<= 12;

        reg_value = ((exp & 0xf) << 12) | (mantissa & 0xfff);
        WICED_BT_TRACE("Threshold register value you set is: %d\r\n", reg_value);
    }

    return reg_value;
}

/******************************************************************************
* Function Name: opt3002_set_low_threshold
***************************************************************************//**
* Set low threshold.
*
* \param  optical_power Low threshold value.
*
* \return None
******************************************************************************/
void opt3002_set_low_threshold(uint32_t optical_power)
{
    uint16_t low_threshold = convert_lux_2_reg_value(optical_power);
    if(low_threshold != 0xffff)
    {
        opt3002_register_write(OPT3002_LOW_THRESHOLD, low_threshold);
    }
}

/******************************************************************************
* Function Name: opt3002_set_upper_threshold
***************************************************************************//**
* Set upper threshold.
*
* \param  optical_power
* Upper threshold value. Range is 0.045 ~ 188000.
*
* \return None
******************************************************************************/
void opt3002_set_upper_threshold(uint32_t optical_power)
{
    uint16_t upper_threshold = convert_lux_2_reg_value(optical_power);
    if(upper_threshold != 0xffff)
    {
        opt3002_register_write(OPT3002_UPPER_THRESHOLD, upper_threshold);
    }
}

/******************************************************************************
* Function Name: opt3002_set_irq_enable
***************************************************************************//**
* Set irq enable.
*
* \param  irq_enable
* Irq enable value.
*
* \return None
******************************************************************************/
void opt3002_set_irq_enable(uint8_t irq_enable)
{
 //   opt3002_reg_info.reg_addr = OPT3002_INTERRUPT_ENABLE;
 //   opt3002_reg_info.reg_value = irq_enable;
 //   wiced_hal_i2c_write( (uint8_t*)&opt3002_reg_info, 0x0002, OPT3002_ADDRESS1);
}
