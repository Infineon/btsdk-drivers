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

/**************************************************************************//**
* \file <opt3002.h>
* List of parameters and defined functions needed to access the
* OPT3002 light sensor driver.
*
******************************************************************************/

#ifndef OPT3002_H
#define OPT3002_H

#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"

#ifndef WICED_HAL_GPIO_PIN_UNUSED
#define WICED_HAL_GPIO_PIN_UNUSED 0xFF
#endif

/**
* \addtogroup
* \ingroup HardwareDrivers
* @{
*
* Defines OPT3002 sensor driver to facilitate interfacing
* with various components of the hardware.
*/

#define OPT3002_ADDRESS1           0x44    /**< sensor addr: 0b01001010                 */

typedef enum
{
    OPT3002_RESULT_REGISTER,        /**< Result register addr          */
    OPT3002_CFG_REGISTER,           /**< Configuration register addr   */
    OPT3002_LOW_THRESHOLD,          /**< Low threshold register addr   */
    OPT3002_UPPER_THRESHOLD,        /**< Upper threshold register addr */
    OPT3002_MANUFACTURER_ID = 0x7E  /**< Manufacturer id register addr */
} OPT3002_REG_t;

#define OPT3002_CFG_RANGE_NUMBER    0x0C  // auto
#define OPT3002_CFG_CONVERSION_TIME 0x00  // 0 -> 100 ms, 1 -> 800 ms
#define OPT3002_CFG_CONVERSION_MODE 0x03  // continuous
#define OPT3002_CFG_LATCH           0x00  // no need to clear flags
#define OPT3002_CFG_POLARITY        0x00  // interrupt pin pulls low
#define OPT3002_CFG_MASK_EXP        0x00  // don't mask exponent field in result
#define OPT3002_CFG_FAULT_TH        0x00  // interrupt on fault count of 1

#define OPT3002_CONFIG_DEFAULT      0xC600

typedef union
{
    struct
    {
		uint16_t fault_threshold : 2;
		uint16_t mask_exponent : 1;
		uint16_t interrupt_polarity : 1;
		uint16_t latch_type : 1;
		uint16_t flag_low : 1;
		uint16_t flag_high : 1;
		uint16_t conversion_ready : 1;
		uint16_t overflow : 1;
		uint16_t mode_of_conversion : 2;
		uint16_t conversion_time : 1;
		uint16_t range_number : 4;
    };

    uint16_t data;
} opt3002_config_reg_t;

typedef struct
{
    uint8_t  reg_addr;
    uint8_t reg_value_hi;
    uint8_t reg_value_lo;
} opt3002_reg_info_t;

typedef struct
{
    uint16_t    scl_pin;                    /**< Scl pin definition */
    uint16_t    sda_pin;                    /**< Sda pin definition */
    uint16_t    irq_pin;                    /**< Pin used to receive interrupt signal from light sensor */
    uint16_t    cfg_reg_value;              /**< Configuration register value */
    uint16_t     upper_threshold_reg_value;  /**< Upper threshold register value */
    uint16_t     low_threshold_reg_value;    /**< Low threshold register value */
    uint8_t     irq_enable_reg_value;       /**< Irq enable register value */
    uint8_t     threshold_timer_reg_value;  /**< Delay time for asserting irq pin when light level exceeds threshold */
}opt3002_user_set_t;


/******************************************************************************
* Function Name: opt3002_int_clean
***************************************************************************//**
* Clean interrupt status.
*
* \param  None
*
* \return None
******************************************************************************/
void opt3002_int_clean (void);

/******************************************************************************
* Function Name: opt3002_register_read
***************************************************************************//**
* read a 16-bit register.
*
* \param  OPT3002_REG_t register id
*
* \return 16-bit register value
******************************************************************************/
uint16_t opt3002_register_read (OPT3002_REG_t reg);

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
uint8_t opt3002_register_write (OPT3002_REG_t reg, uint16_t value);

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
void opt3002_init(opt3002_user_set_t *opt3002_usr_set, void (*user_fn)(void*, uint8_t), void* usr_data);

/******************************************************************************
* Function Name: opt3002_read_ambient_light
***************************************************************************//**
* Read light sensor status.
*
* \param  None.
*
* \return lux_value.
* Resolution is 0.01, Max range is 167772.14(3octets).
******************************************************************************/
uint32_t opt3002_read_ambient_light(void);

/******************************************************************************
* Function Name: opt3002_set_low_threshold
***************************************************************************//**
* Set low threshold.
*
* \param  lux_value
* Low threshold value. Range is 0.045 ~ 188000.
*
* \return None
******************************************************************************/
void opt3002_set_low_threshold(uint32_t lux_value);

/******************************************************************************
* Function Name: opt3002_set_upper_threshold
***************************************************************************//**
* Set upper threshold.
*
* \param  lux_value
* Upper threshold value. Range is 0.045 ~ 188000.
*
* \return None
******************************************************************************/
void opt3002_set_upper_threshold(uint32_t lux_value);

/******************************************************************************
* Function Name: opt3002_set_irq_enable
***************************************************************************//**
* Set irq enable.
*
* \param  irq_enable
*
* \return None
******************************************************************************/
void opt3002_set_irq_enable(uint8_t irq_enable);


/* @} */

#endif
