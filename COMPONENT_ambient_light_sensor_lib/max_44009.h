/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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

/**************************************************************************//**
* \file <max_44009.h>
* List of parameters and defined functions needed to access the
* max44009 light sensor driver.
*
******************************************************************************/

#ifndef MAX44009_H
#define MAX44009_H

#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"

/**
* \addtogroup
* \ingroup HardwareDrivers
* @{
*
* Defines max44009 sensor driver to facilitate interfacing
* with various components of the hardware.
*/

#define MAX44009_ADDRESS1           0x4A    /**< sensor addr: 0b01001010                 */

#define MAX44009_INTERRUPT_STATUS   0x00    /**< Interrupt status register addr          */
#define MAX44009_INTERRUPT_ENABLE   0x01    /**< Interrupt status enable register addr   */
#define MAX44009_CFG_REGISTER       0x02    /**< Configuration register addr             */
#define MAX44009_LUX_HIGH_REGISTER  0x03    /**< Lux high byte register addr             */
#define MAX44009_LUX_LOW_REGISTER   0x04    /**< Lux low byte register addr              */
#define MAX44009_UPPER_THRESHOLD    0x05    /**< Upper threshold high byte register addr */
#define MAX44009_LOW_THRESHOLD      0x06    /**< Low threshold high byte register addr   */
#define MAX44009_THRESHOLD_TIMER    0x07    /**< Threshold timer register addr           */

typedef struct
{
    uint8_t reg_addr;
    uint8_t reg_value;
}max44009_reg_info_t;

typedef struct
{
    uint16_t    scl_pin;                    /**< Scl pin definition */
    uint16_t    sda_pin;                    /**< Sda pin definition */
    uint16_t    irq_pin;                    /**< Pin used to receive interrupt signal from light sensor */
    uint8_t     irq_enable_reg_value;       /**< Irq enable register value */
    uint8_t     cfg_reg_value;              /**< Configuration register value */
    uint8_t     upper_threshold_reg_value;  /**< Upper threshold register value */
    uint8_t     low_threshold_reg_value;    /**< Low threshold register value */
    uint8_t     threshold_timer_reg_value;  /**< Delay time for asserting irq pin when light level exceeds threshold */
}max44009_user_set_t;


/******************************************************************************
* Function Name: max44009_int_clean
***************************************************************************//**
* Clean interrupt status.
*
* \param  None
*
* \return None
******************************************************************************/
void max44009_int_clean (void);

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
void max44009_init(max44009_user_set_t *max44009_usr_set, void (*user_fn)(void*, uint8_t), void* usr_data);

/******************************************************************************
* Function Name: max44009_read_ambient_light
***************************************************************************//**
* Read light sensor status.
*
* \param  None.
*
* \return lux_value.
* Resolution is 0.01, Max range is 167772.14(3octets).
******************************************************************************/
uint32_t max44009_read_ambient_light(void);

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
void max44009_set_low_threshold(uint32_t lux_value);

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
void max44009_set_upper_threshold(uint32_t lux_value);

/******************************************************************************
* Function Name: max44009_set_irq_enable
***************************************************************************//**
* Set irq enable.
*
* \param  irq_enable
*
* \return None
******************************************************************************/
void max44009_set_irq_enable(uint8_t irq_enable);


/* @} */

#endif
