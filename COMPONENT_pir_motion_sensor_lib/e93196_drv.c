/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
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

#include "e93196_drv.h"
#include "wiced_hal_gpio.h"
#include "e93196.h"

/******************************************************************************
* Function Name: e93196_serin_high
***************************************************************************//**
* SERIN output high.
*
* \param  serin_pin
* Serial Input configure pin
*
* \return None
******************************************************************************/
void e93196_serin_high (uint16_t serin_pin)
{
    wiced_hal_gpio_configure_pin( serin_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH );
}

/******************************************************************************
* Function Name: e93196_serin_low
***************************************************************************//**
* SERIN output low.
*
* \param  serin_pin
* Serial Input configure pin
*
* \return None
******************************************************************************/
void e93196_serin_low (uint16_t serin_pin)
{
    wiced_hal_gpio_configure_pin( serin_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW );
}

/******************************************************************************
* Function Name: e93196_doci_cfg_output
***************************************************************************//**
* DOCI configure output mode.
*
* \param  doci_pin
* Interrupt/Data output Clock input configure pin
*
* \return None
******************************************************************************/
void e93196_doci_cfg_output (uint16_t doci_pin)
{
    wiced_hal_gpio_configure_pin( doci_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW );
}

/******************************************************************************
* Function Name: e93196_doci_high
***************************************************************************//**
* DOCI high.
*
* \param  doci_pin
* Interrupt/Data output Clock input configure pin
*
* \return None
******************************************************************************/
void e93196_doci_high (uint16_t doci_pin)
{
    wiced_hal_gpio_configure_pin( doci_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH );
}

/******************************************************************************
* Function Name: e93196_doci_low
***************************************************************************//**
* DOCI low.
*
* \param  doci_pin
* Interrupt/Data output Clock input configure pin
*
* \return None
******************************************************************************/
void e93196_doci_low (uint16_t doci_pin)
{
    wiced_hal_gpio_configure_pin( doci_pin, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW );
}

/******************************************************************************
* Function Name: e93196_doci_cfg_input
***************************************************************************//**
* DOCI configure input mode.
*
* \param  doci_pin
* Interrupt/Data output Clock input configure pin
*
* \return None
******************************************************************************/
void e93196_doci_cfg_input (uint16_t doci_pin)
{
    wiced_hal_gpio_configure_pin( doci_pin, GPIO_INPUT_ENABLE, GPIO_PIN_OUTPUT_LOW );
}

/******************************************************************************
* Function Name: e93196_doci_ipu
***************************************************************************//**
* DOCI input pull up.
*
* \param  doci_pin
* Interrupt/Data output Clock input configure pin
*
* \return None
******************************************************************************/
void e93196_doci_ipu (uint16_t doci_pin)
{
    wiced_hal_gpio_configure_pin( doci_pin, GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE, GPIO_PIN_OUTPUT_HIGH );
}

/******************************************************************************
* Function Name: e93196_doci_ipd
***************************************************************************//**
* DOCI input pull down.
*
* \param  doci_pin
* Interrupt/Data output Clock input configure pin
*
* \return None
******************************************************************************/
void e93196_doci_ipd (uint16_t doci_pin)
{
    wiced_hal_gpio_configure_pin( doci_pin, GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_RISING_EDGE, GPIO_PIN_OUTPUT_LOW );
}

/******************************************************************************
* Function Name: e93196_doci_read
***************************************************************************//**
* DOCI input status read.
*
* \param doci_pin
* Interrupt/Data output Clock input configure pin
*
* \return None
******************************************************************************/
uint8_t e93196_doci_read (uint16_t doci_pin)
{
    return wiced_hal_gpio_get_pin_input_status(doci_pin);
}
