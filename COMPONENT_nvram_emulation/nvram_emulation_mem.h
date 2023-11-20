/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef NVRAM_EMULATION_H
#define NVRAM_EMULATION_H

/*! @{ */
/*******************************************************************************
*  Include
*******************************************************************************/
#include <wiced.h>
#include <stddef.h>
#include "wiced_hal_nvram.h"

/*******************************************************************************
*  Public Macro and Literal Definition
*******************************************************************************/
#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

/*******************************************************************************
*  Public Typedef and Enum
*******************************************************************************/

/**********************************************************************
* nvram_emulation_entry_t - key-length-value storage for nvram item
***********************************************************************/
typedef struct
{
    uint16_t vs_id;
    uint16_t length;
    uint8_t p_data[];
} nvram_emulation_entry_t;

/*******************************************************************************
*  Public Structure
*******************************************************************************/

/*******************************************************************************
*  Extern Variable
*******************************************************************************/

/*******************************************************************************
*  Public Function Declaration
*******************************************************************************/

/**********************************************************************
* nvram_emulation_mem_init - initialize pools or heap for allocation/free
* init list of allocated buffers used to search for items
* also init rtos mutex to keep list access sequential
***********************************************************************/
wiced_bool_t nvram_emulation_mem_init(void);

/**********************************************************************
* nvram_emulation_entry_allocate - allocate an entry
* return the entry pointer or NULL if allocation fails
***********************************************************************/
nvram_emulation_entry_t *nvram_emulation_entry_allocate(uint32_t size);

/**********************************************************************
* nvram_emulation_entry_free - free the storage for an entry
***********************************************************************/
void nvram_emulation_entry_free(nvram_emulation_entry_t *p_entry);

/**********************************************************************
* nvram_emulation_peek_first_entry - get pointer to first allocated entry
***********************************************************************/
nvram_emulation_entry_t *nvram_emulation_peek_first_entry(void);

/**********************************************************************
* nvram_emulation_peek_next_entry - using entry pointer, get pointer to next allocated entry
***********************************************************************/
nvram_emulation_entry_t *nvram_emulation_peek_next_entry(nvram_emulation_entry_t *p_buffer);


/**********************************************************************
* nvram_emulation_read - same as wiced_hal_read_nvram
***********************************************************************/
uint16_t nvram_emulation_read(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status);

/**********************************************************************
* nvram_emulation_write - same as wiced_hal_write_nvram
***********************************************************************/
uint16_t nvram_emulation_write(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status);

/**********************************************************************
* nvram_emulation_delete - same as wiced_hal_delete_nvram
***********************************************************************/
void nvram_emulation_delete(uint16_t vs_id, wiced_result_t *p_status);

/* macros to redefine wiced hal nvram APIs to nvram_emulation*/
#define wiced_hal_read_nvram(vs_id, data_length, p_data, p_status) nvram_emulation_read(vs_id, data_length, p_data, p_status)
#define wiced_hal_write_nvram(vs_id, data_length, p_data, p_status) nvram_emulation_write(vs_id, data_length, p_data, p_status)
#define wiced_hal_delete_nvram(vs_id, p_status) nvram_emulation_delete(vs_id, p_status)

/** for debug */
void nvram_emulation_print_state(void);


/* @} */
#endif // NVRAM_EMULATION_H
