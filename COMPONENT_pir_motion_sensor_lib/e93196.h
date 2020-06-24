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
* \file <e93196.h>
* List of parameters and defined functions needed to access the
* e93196 motion sensor driver.
*
******************************************************************************/

#ifndef __E93196_H
#define __E93196_H

#include "e93196_drv.h"

/**
* \addtogroup
* \ingroup HardwareDrivers
* @{
*
* Defines e93196 motion sensor driver to facilitate interfacing
* with various components of the hardware.
*/

/**********************************************************************************************************
  registers offset and mask
**********************************************************************************************************/
#define E93196_PIR_SFT              ( 39 )
#define E93196_OUT_SFT              ( 25 )
#define E93196_SENSE_SFT            ( 17 )
#define E93196_BLIND_SFT            ( 13 )
#define E93196_PULSE_SFT            ( 11 )
#define E93196_WINDOW_SFT           ( 9 )
#define E93196_MOVE_SFT             ( 8 )
#define E93196_INT_SFT              ( 7 )
#define E93196_ADC_SFT              ( 5 )
#define E93196_POWER_SFT            ( 4 )
#define E93196_SELFTEST_SFT         ( 3 )
#define E93196_CAPA_SFT             ( 2 )
#define E93196_TESTMODE_SFT         ( 0 )

#define E93196_PIR_MASK             ( 0x01 )
#define E93196_OUT_MASK             ( 0x3FFF )
#define E93196_SENSE_MASK           ( 0xFF )
#define E93196_BLIND_MASK           ( 0xF )
#define E93196_PULSE_MASK           ( 0x03 )
#define E93196_WINDOW_MASK          ( 0x03 )
#define E93196_MOVE_MASK            ( 0x01 )
#define E93196_INT_MASK             ( 0x01 )
#define E93196_ADC_MASK             ( 0x03 )
#define E93196_POWER_MASK           ( 0x01 )
#define E93196_SELFTEST_MASK        ( 0x01 )
#define E93196_CAPA_MASK            ( 0x01 )
#define E93196_TESTMODE_MASK        ( 0x03 )



/**********************************************************************************************************
data definition
**********************************************************************************************************/

typedef struct e93196_writereg
{
    uint8_t sensitivity;                                                /**< [24:17]sensitivity                   */
    uint8_t blind_time;                                                 /**< [16:13]blind time                    */
    uint8_t pulse_cnt;                                                  /**< [12:11]pulse count                   */
    uint8_t window_time;                                                /**< [10:9]window time                    */
    uint8_t move_dete_en;                                               /**< [8]move detect enable                */
    uint8_t int_src;                                                    /**< [7]irq source                        */
    uint8_t adc_filter;                                                 /**< [6:5]ADC filter                      */
    uint8_t power_en;                                                   /**< [4]power enable                      */
    uint8_t self_test_en;                                               /**< [3]selftest enable                   */
    uint8_t capa;	                                                    /**< [2]selftest capacity                 */
    uint8_t test_mode;                                                  /**< [1:0]reserved                        */
}e93196_write_reg_t;                                                    /**< define E931.96 input data type       */


typedef struct
{
    uint8_t  pir_range;                                                 /**< [39]PIR output range                 */
    uint32_t pir_out;                                                   /**< [38:25]PIR output voltage            */
    uint8_t  sensitivity;                                               /**< [24:17]sensitivity                   */
    uint8_t  blind_time;                                                /**< [16:13]blind time                    */
    uint8_t  pulse_cnt;                                                 /**< [12:11]pulse count                   */
    uint8_t  window_time;                                               /**< [10:9]window time                    */
    uint8_t  move_dete_en;                                              /**< [8]move detect enable                */
    uint8_t  int_src;                                                   /**< [7]irq source                        */
    uint8_t  adc_filter;                                                /**< [6:5]ADC filter                      */
    uint8_t  power_en;                                                  /**< [4]power enable                      */
    uint8_t  self_test_en;                                              /**< [3]selftest enable                   */
    uint8_t  capa;                                                      /**< [2]selftest capacity                 */
    uint8_t  test_mode;                                                 /**< [1:0]reserved                        */
}e93196_read_reg_t;                                                     /**< define E931.96 output data type      */

typedef struct
{
    uint16_t              doci_pin;                                     /**< doci pin definition                  */
    uint16_t              serin_pin;                                    /**< serin pin definition                 */
    e93196_write_reg_t    e93196_init_reg;                              /**< sensor register init config          */
}e93196_usr_cfg_t;                                                      /**< user config set                      */


/******************************************************************************
* Function Name: e93196_init
***************************************************************************//**
* Initializes the 93196 motion sensor.
*
* \param e93196_usr_cfg_t *p_e93196_usr_cfg
* config user structure.
*
*       uint16_t              doci_pin        - doci pin definition
*
*       uint16_t              serin_pin       - serin pin definition
*
*       e93196_write_reg_t  e93196_init_reg - sensor register init configure
*
* \param user_fn - Points to the function to call when the interrupt
* comes .Below is the description of the arguments received by the cb.
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
void e93196_init ( e93196_usr_cfg_t *p_e93196_usr_cfg, void (*user_fn)(void*, uint8_t), void* usr_data );

/******************************************************************************
* Function Name: e93196_int_clean
***************************************************************************//**
* Clean interrupt status.
*
* \param doci_pin.
* Interrupt/Data output Clock input config pin
*
* \return None
******************************************************************************/
void e93196_int_clean(uint16_t doci_pin);

/******************************************************************************
* Function Name: e93196_sensitivity_set
***************************************************************************//**
* write sensitivity into register.
*
* \param sensitivity
* The sensitivity data to write.
*
* \return None
******************************************************************************/
void e93196_sensitivity_set (uint8_t sensitivity);

/******************************************************************************
* Function Name: e93196_blind_time_set
***************************************************************************//**
* write blind time into register.
*
* \param blind_time
* The sensitivity data to write.
*
* \return None
******************************************************************************/
void e93196_blind_time_set (uint8_t blind_time);

/******************************************************************************
* Function Name: e93196_pir_out_read
***************************************************************************//**
* read PIR Output voltage.
*
* \param void
*
* \return pir_out
* PIR output voltage
******************************************************************************/
uint32_t e93196_pir_out_read (void);

/******************************************************************************
* Function Name: e93196_reg_write
***************************************************************************//**
* write data into register.
*
* \param e93196_write_reg
* The input data to write.
*
* \return None
******************************************************************************/
void e93196_reg_write (e93196_write_reg_t *p_e93196_write_reg);

/******************************************************************************
* Function Name: e93196_reg_read
***************************************************************************//**
* read data from register.
*
* \param e93196_read_reg
* The output data to read.
*
* \return None
******************************************************************************/
void e93196_reg_read (e93196_read_reg_t *p_e93196_read_reg);


/* @} */

#endif                                                                  /* __E93196_H  */
