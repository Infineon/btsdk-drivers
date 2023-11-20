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
#include "e93196.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"

#define E93196_DBG_PRINT_EN 1

/**********************************************************************************************************
Variables Definitions
**********************************************************************************************************/
e93196_read_reg_t  e93196_read_reg;                                                                         /* E931.96 read reg                         */
static uint16_t serin_pin, doci_pin;


/**********************************************************************************************************
Function Declarations
**********************************************************************************************************/
void e93196_data_read(uint16_t doci_pin, uint32_t *p_ui_data);                                     /* E931.96 data read                        */
void e93196_data_write(uint16_t serin_pin, uint32_t ui_data);                                      /* E931.96 data write                       */
void e93196_read_data_trans(e93196_read_reg_t  *p_e93196_read_reg, uint32_t *p_ui_read_data);   /* E931.96 read data format translate       */
void e93196_write_data_trans(uint32_t *p_ui_write_data, e93196_write_reg_t *p_e93196_write_reg);/* E931.96 write data format translate      */
void e93196_sys_tick_delay(uint32_t delay_us);                                                   /* E931.96 delay                            */
void e93196_reg_write(e93196_write_reg_t *p_e93196_write_reg);                                      /* write data into register                 */
void e93196_reg_read(e93196_read_reg_t *p_e93196_read_reg);                                          /* read data from register                  */

#if E93196_DBG_PRINT_EN
void e93196_write_data_print(e93196_write_reg_t *p_e93196_write_reg);
void e93196_read_data_print(e93196_read_reg_t *p_e93196_read_reg);
#endif
extern void utilslib_delayUs(UINT32 delay);

/******************************************************************************
* Function Name: e93196_init
***************************************************************************//**
* Initializes the 93196 motion sensor.
*
* \param e93196_usr_cfg_t *p_e93196_usr_cfg
* configure user structure.
*
*       uint16_t        doci_pin                - doci pin definition
*
*       uint16_t        serin_pin               - serin pin definition
*
*       e93196_write_reg_t  e93196_init_reg     - sensor register init configure
*
* \param user_fn - Points to the function to call when the interrupt
* comes .Below is the description of the arguments received by the cb.
*
*       void* user_data  - User data provided when interrupt is being registered
*                        using wiced_hal_gpio_register_pin_for_interrupt(...)
*
*       uint8_t port_pin - Number of the pin causing the interrupt
*
* \param usrData
* Will be passed back to user_fn as-is. Typically NULL.
*
* \return None
******************************************************************************/
void e93196_init(e93196_usr_cfg_t *p_e93196_usr_cfg, void (*user_fn)(void*, uint8_t), void* usr_data)
{
    serin_pin = p_e93196_usr_cfg->serin_pin;
    doci_pin  = p_e93196_usr_cfg->doci_pin;
    e93196_serin_low(serin_pin);
    e93196_doci_cfg_input(doci_pin);                                        /* DOCI  input mode   */

    e93196_reg_write(&p_e93196_usr_cfg->e93196_init_reg);                   /* E931.96 reg init   */
    e93196_reg_read(&e93196_read_reg);                                      /* E931.96 data read  */

    if (user_fn != NULL)
    {
        wiced_hal_gpio_configure_pin(doci_pin, (GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_LEVEL_HIGH), GPIO_PIN_OUTPUT_LOW);
        wiced_hal_gpio_register_pin_for_interrupt(doci_pin, user_fn, usr_data);
    }

}


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
void e93196_int_clean(uint16_t doci_pin)
{
    e93196_doci_low(doci_pin);                                              /* clear interrupt */
    wiced_hal_gpio_configure_pin(doci_pin, GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_LEVEL_HIGH, GPIO_PIN_OUTPUT_LOW);
}


/******************************************************************************
* Function Name: e93196_reg_write
***************************************************************************//**
* write data into register.
*
* \param p_e93196_write_reg
* The input data to write.
*
* \return None
******************************************************************************/
void e93196_reg_write(e93196_write_reg_t *p_e93196_write_reg)
{
    uint32_t ui_data;

    e93196_write_data_trans(&ui_data, p_e93196_write_reg);                  /* write_data transferred to bit stream */
    e93196_data_write(serin_pin, ui_data);

#if E93196_DBG_PRINT_EN
    WICED_BT_TRACE("Write Data = %d\r\n", ui_data);
    e93196_write_data_print(p_e93196_write_reg);
#endif
}



/******************************************************************************
* Function Name: e93196_reg_read
***************************************************************************//**
* read data from register.
*
* \param p_e93196_read_reg
* The output data to read.
*
* \return None
******************************************************************************/
void e93196_reg_read(e93196_read_reg_t *p_e93196_read_reg)
{
    uint32_t ui_data[2] = {0, 0};

    e93196_data_read(doci_pin, ui_data);                                    /* E931.96 data read#if E93196_DBG_PRINT_EN */
    e93196_read_data_trans(p_e93196_read_reg, ui_data);                     /* bit steam transferred to structure */

#if E93196_DBG_PRINT_EN
    e93196_read_data_print(p_e93196_read_reg);
#endif
}

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
void e93196_sensitivity_set(uint8_t sensitivity)
{
    e93196_write_reg_t write_reg;
    e93196_read_reg_t read_reg;

    e93196_reg_read(&read_reg);
    memcpy(&write_reg, &(read_reg.sensitivity), sizeof(e93196_write_reg_t));
    write_reg.sensitivity = sensitivity;
    e93196_reg_write(&write_reg);                                           /* write_data transferred to bit stream */

#if E93196_DBG_PRINT_EN
    e93196_reg_read(&read_reg);
#endif
}

/******************************************************************************
* Function Name: e93196_blind_time_set
***************************************************************************//**
* write blind_time into register.
*
* \param blind_time
* The sensitivity data to write.
*
* \return None
******************************************************************************/
void e93196_blind_time_set(uint8_t blind_time)
{
    e93196_write_reg_t write_reg;
    e93196_read_reg_t read_reg;

    e93196_reg_read(&read_reg);
    memcpy(&write_reg, &(read_reg.sensitivity), sizeof(e93196_write_reg_t));
    write_reg.blind_time = blind_time;
    e93196_reg_write(&write_reg);                                           /* write_data transferred to bit stream */

#if E93196_DBG_PRINT_EN
    e93196_reg_read(&read_reg);
#endif
}

/******************************************************************************
* Function Name: e93196_pir_out_read
***************************************************************************//**
* read PIR Output voltage.
*
* \param void
*
* \return ulPirOut
* PIR output voltage
******************************************************************************/
uint32_t e93196_pir_out_read(void)
{
    e93196_read_reg_t read_reg;

    e93196_reg_read(&read_reg);
    return read_reg.pir_out;
}


/******************************************************************************
* Function Name: e93196_data_read
***************************************************************************//**
* read data from register.
*
* \param doci_pin.
* Interrupt/Data output Clock input configure pin
*
* \param p_ui_data
* the data pointer finished read.
*
* \return None
******************************************************************************/
void e93196_data_read(uint16_t doci_pin, uint32_t *p_ui_data)
{
    int i;
    uint64_t ui_data64 = 0;

    e93196_doci_high(doci_pin);

    e93196_sys_tick_delay(122);
    e93196_doci_low(doci_pin);

    for (i = 0; i < 40; i++)
    {                                                                       /* low 32 bit */
        e93196_doci_high(doci_pin);                                         /* get ready to read data */

        e93196_doci_ipu(doci_pin);
        ui_data64 <<= 1;

        if (1 == e93196_doci_read(doci_pin))
        {
            ui_data64 |= 0x01;                                              /* reset data */
            e93196_doci_high(doci_pin);
        }
        else
        {
            ui_data64 &= ~0x01;                                             /* clear data */
            e93196_doci_low(doci_pin);
        }

        e93196_doci_low(doci_pin);
    }

    e93196_doci_ipd(doci_pin);

    p_ui_data[0] = (ui_data64  >> 32) & 0xff;
    p_ui_data[1] = (ui_data64) &0xffffffff;
}


/******************************************************************************
* Function Name: e93196_data_write
***************************************************************************//**
* read data from register.
*
* \param serin_pin
* Serial Input  configure pin
*
* \param ui_data
* data need to write.
*
* \return None
******************************************************************************/
void e93196_data_write(uint16_t serin_pin, uint32_t ui_data)
{
    int i;

    for (i = 0; i < 25; i++)
    {                                                                       /* traverse data            */
        e93196_serin_low(serin_pin);
        e93196_serin_high(serin_pin);                                       /* get ready to write data  */

        if (ui_data & (1 << (24 - i)))                                     /* traverse data into MSB   */
        {
            e93196_serin_high(serin_pin);
        }
        else
        {
            e93196_serin_low(serin_pin);
        }
        e93196_sys_tick_delay(60);
    }

    e93196_serin_low(serin_pin);
    e93196_sys_tick_delay(56);

}


/******************************************************************************
* Function Name: e93196_read_data_trans
***************************************************************************//**
* translate the read data into struct format.
*
* \param p_e93196_read_reg
* target struct pointer.
*
* \param p_ui_read_data
* the original read data.
*
* \return None
******************************************************************************/
void e93196_read_data_trans(e93196_read_reg_t *p_e93196_read_reg, uint32_t *p_ui_read_data)
{
    uint64_t ui_data64;
    ui_data64 = ((uint64_t)p_ui_read_data[0] << 32) | (p_ui_read_data[1] << 0);

    p_e93196_read_reg->pir_range    = (ui_data64 >> E93196_PIR_SFT     ) & E93196_PIR_MASK;
    p_e93196_read_reg->pir_out      = (ui_data64 >> E93196_OUT_SFT     ) & E93196_OUT_MASK;
    p_e93196_read_reg->sensitivity  = (ui_data64 >> E93196_SENSE_SFT   ) & E93196_SENSE_MASK;
    p_e93196_read_reg->blind_time   = (ui_data64 >> E93196_BLIND_SFT   ) & E93196_BLIND_MASK;
    p_e93196_read_reg->pulse_cnt    = (ui_data64 >> E93196_PULSE_SFT   ) & E93196_PULSE_MASK;
    p_e93196_read_reg->window_time  = (ui_data64 >> E93196_WINDOW_SFT  ) & E93196_WINDOW_MASK;
    p_e93196_read_reg->move_dete_en = (ui_data64 >> E93196_MOVE_SFT    ) & E93196_MOVE_MASK;
    p_e93196_read_reg->int_src      = (ui_data64 >> E93196_INT_SFT     ) & E93196_INT_MASK;
    p_e93196_read_reg->adc_filter   = (ui_data64 >> E93196_ADC_SFT     ) & E93196_ADC_MASK;
    p_e93196_read_reg->power_en     = (ui_data64 >> E93196_POWER_SFT   ) & E93196_POWER_MASK;
    p_e93196_read_reg->self_test_en = (ui_data64 >> E93196_SELFTEST_SFT) & E93196_SELFTEST_MASK;
    p_e93196_read_reg->capa         = (ui_data64 >> E93196_CAPA_SFT    ) & E93196_CAPA_MASK;
    p_e93196_read_reg->test_mode    = (ui_data64 >> E93196_TESTMODE_SFT) & E93196_TESTMODE_MASK;
}


/******************************************************************************
* Function Name: e93196_write_data_trans
***************************************************************************//**
* translate the write data into structure format.
*
* \param p_ui_write_data
* translate target.
*
* \param p_e93196_write_reg
* original struct pointer.
*
* \return None
******************************************************************************/
void e93196_write_data_trans(uint32_t *p_ui_write_data, e93196_write_reg_t *p_e93196_write_reg)
{
    uint32_t ui_data = 0;

    ui_data |= p_e93196_write_reg->sensitivity  << E93196_SENSE_SFT;
    ui_data |= p_e93196_write_reg->blind_time   << E93196_BLIND_SFT;
    ui_data |= p_e93196_write_reg->pulse_cnt    << E93196_PULSE_SFT;
    ui_data |= p_e93196_write_reg->window_time  << E93196_WINDOW_SFT;
    ui_data |= p_e93196_write_reg->move_dete_en << E93196_MOVE_SFT;
    ui_data |= p_e93196_write_reg->int_src      << E93196_INT_SFT;
    ui_data |= p_e93196_write_reg->adc_filter   << E93196_ADC_SFT;
    ui_data |= p_e93196_write_reg->power_en     << E93196_POWER_SFT;
    ui_data |= p_e93196_write_reg->self_test_en << E93196_SELFTEST_SFT;
    ui_data |= p_e93196_write_reg->capa         << E93196_CAPA_SFT;
    ui_data |= p_e93196_write_reg->test_mode    << E93196_TESTMODE_SFT;

    *p_ui_write_data = ui_data;
}


/******************************************************************************
* Function Name: e93196_read_data_print
***************************************************************************//**
* Print the data which read from register.
*
* \param p_e93196_read_reg
* The output data to print.
*
* \return None
******************************************************************************/
void e93196_read_data_print(e93196_read_reg_t *p_e93196_read_reg)
{
    WICED_BT_TRACE("pir_range           = %d\r\n", p_e93196_read_reg->pir_range);               /* data print */
    WICED_BT_TRACE("pir_out             = %d\r\n", p_e93196_read_reg->pir_out);
    WICED_BT_TRACE("sensitivity         = %d\r\n", p_e93196_read_reg->sensitivity);
    WICED_BT_TRACE("blind_time          = %d\r\n", p_e93196_read_reg->blind_time);
    WICED_BT_TRACE("pulse_cnt           = %d\r\n", p_e93196_read_reg->pulse_cnt);
    WICED_BT_TRACE("window_time         = %d\r\n", p_e93196_read_reg->window_time);
    WICED_BT_TRACE("move_dete_en        = %d\r\n", p_e93196_read_reg->move_dete_en);
    WICED_BT_TRACE("int_src             = %d\r\n", p_e93196_read_reg->int_src);
    WICED_BT_TRACE("adc_filter          = %d\r\n", p_e93196_read_reg->adc_filter);
    WICED_BT_TRACE("power_en            = %d\r\n", p_e93196_read_reg->power_en);
    WICED_BT_TRACE("self_test_en        = %d\r\n", p_e93196_read_reg->self_test_en);
    WICED_BT_TRACE("capa                = %d\r\n", p_e93196_read_reg->capa);
    WICED_BT_TRACE("test_mode           = %d\r\n", p_e93196_read_reg->test_mode);

    WICED_BT_TRACE("\r\n");
}

/******************************************************************************
* Function Name: e93196_write_data_print
***************************************************************************//**
* Print the data which ready to write.
*
* \param p_e93196_write_reg
* The input data to print.
*
* \return None
******************************************************************************/
void e93196_write_data_print(e93196_write_reg_t *p_e93196_write_reg)
{
    WICED_BT_TRACE("sensitivity         = %d\r\n", p_e93196_write_reg->sensitivity);            /* data print */
    WICED_BT_TRACE("blind_time          = %d\r\n", p_e93196_write_reg->blind_time);
    WICED_BT_TRACE("pulse_cnt           = %d\r\n", p_e93196_write_reg->pulse_cnt);
    WICED_BT_TRACE("window_time         = %d\r\n", p_e93196_write_reg->window_time);
    WICED_BT_TRACE("move_dete_en        = %d\r\n", p_e93196_write_reg->move_dete_en);
    WICED_BT_TRACE("int_src             = %d\r\n", p_e93196_write_reg->int_src);
    WICED_BT_TRACE("adc_filter          = %d\r\n", p_e93196_write_reg->adc_filter);
    WICED_BT_TRACE("power_en            = %d\r\n", p_e93196_write_reg->power_en);
    WICED_BT_TRACE("self_test_en        = %d\r\n", p_e93196_write_reg->self_test_en);
    WICED_BT_TRACE("capa                = %d\r\n", p_e93196_write_reg->capa);
    WICED_BT_TRACE("test_mode           = %d\r\n", p_e93196_write_reg->test_mode);

    WICED_BT_TRACE("\r\n");
}


/******************************************************************************
* Function Name: e93196_sys_tick_delay
***************************************************************************//**
* software delay.
*
* \param delay_us
* dealy time(unit: us).
*
* \return None
******************************************************************************/
void e93196_sys_tick_delay(uint32_t delay_us)
{
	utilslib_delayUs(delay_us);
}
