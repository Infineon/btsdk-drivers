/*
 * Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
*  Include
*******************************************************************************/
#include "wiced.h"
#include "wiced_memory.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "nvram_emulation_mem.h"
#include "slist.h"

/*******************************************************************************
*  Global Variable
*******************************************************************************/
int nvram_emulation_initialized = 0;
/*******************************************************************************
*  Private Macro and Literal Definition
*******************************************************************************/
#define __num_of(a) (sizeof(a)/sizeof(a[0]))

// declare as weak to allow app to override default config
// ordered by size, small to large
__WEAK nvram_emulation_mem_buf_cfg_t nvram_emulation_mem_cfg[] =
{
    // size,    num
    {32,        2},
    {80,        2},
    {284,       2},
    {572,       1},
};

/*******************************************************************************
*  Static Variable
*******************************************************************************/
static struct
{
#if BTSTACK_VER < 0x03000001
    wiced_bt_buffer_pool_t *p_pool[__num_of(nvram_emulation_mem_cfg)];
#else
    wiced_bt_heap_t *p_heap;
#endif
} nvram_emulation_mem_info = {0};

typedef struct
{
    slist_node_t node;
    nvram_emulation_entry_t entry;
} nvram_emulation_buffer_list_t;
nvram_emulation_buffer_list_t nvram_emulation_buffer_list;

/*******************************************************************************
*  Declaration of Static Functions
*******************************************************************************/

/*******************************************************************************
*  Global Function for Outside
*******************************************************************************/

wiced_bool_t nvram_emulation_mem_init(void)
{
    uint16_t i;
    uint16_t num;
    uint32_t buffer_size;
    uint32_t heap_size = 0;

    if(nvram_emulation_initialized)
    {
        return WICED_ALREADY_INITIALIZED;
    }
    nvram_emulation_initialized = 1;

#if BTSTACK_VER >= 0x03000001
    /* allocate list of buffer pools */
    for (i = 0 ; i < __num_of(nvram_emulation_mem_cfg); i++)
    {
        buffer_size = nvram_emulation_mem_cfg[i].size + offsetof(nvram_emulation_buffer_list_t, entry.p_data);
        num = nvram_emulation_mem_cfg[i].num;
        heap_size += buffer_size * num;
    }
    nvram_emulation_mem_info.p_heap = wiced_bt_create_heap ("nvram_emu", NULL, heap_size, NULL, FALSE);

#else // BTSTACK_VER < 0x03000001

    for (i = 0 ; i < __num_of(nvram_emulation_mem_cfg); i++)
    {
        buffer_size = nvram_emulation_mem_cfg[i].size + offsetof(nvram_emulation_buffer_list_t, entry.p_data);
        num = nvram_emulation_mem_cfg[i].num;
        nvram_emulation_mem_info.p_pool[i] = wiced_bt_create_pool(buffer_size, num);
        if(nvram_emulation_mem_info.p_pool[i] == NULL)
        {
            return WICED_FALSE;
        }
    }
#endif
    // initialize empty list of buffers
    INIT_SLIST_NODE(&nvram_emulation_buffer_list.node);

    return WICED_TRUE;
}

nvram_emulation_entry_t *nvram_emulation_entry_allocate(uint32_t size)
{
    nvram_emulation_buffer_list_t *p = NULL;
    uint32_t buffer_size = size + offsetof(nvram_emulation_buffer_list_t, entry.p_data);

#if BTSTACK_VER >= 0x03000001
    p = (nvram_emulation_buffer_list_t *)wiced_bt_get_buffer_from_heap(nvram_emulation_mem_info.p_heap, buffer_size);
#else
    uint16_t i;
    for (i = 0 ; i < __num_of(nvram_emulation_mem_cfg); i++)
    {
        if(size < nvram_emulation_mem_cfg[i].size)
        {
            p = (nvram_emulation_buffer_list_t *)wiced_bt_get_buffer_from_pool(nvram_emulation_mem_info.p_pool[i]);

            // if !p spin again on next pool to see if a buffer is available
            if(p != NULL)
            {
                break;
            }
        }
    }
#endif
    if(p == NULL)
    {
        return NULL;
    }
    memset(p, 0, sizeof(buffer_size));
    slist_add_tail(&p->node, &nvram_emulation_buffer_list.node);

    return &p->entry;
}

void nvram_emulation_entry_free(nvram_emulation_entry_t *p_entry)
{
    nvram_emulation_buffer_list_t *p;
    if(p_entry)
    {
        // back up address from p_data to start of struct
        p = slist_entry(p_entry, nvram_emulation_buffer_list_t, entry);
        if(!slist_empty(&nvram_emulation_buffer_list.node))
        {
            slist_del(&p->node, &nvram_emulation_buffer_list.node);
        }
        wiced_bt_free_buffer (p);
    }
}

/******************************************************************************
* slist_empty - tests whether a list is empty
* @head: the list to test.
******************************************************************************/
nvram_emulation_entry_t *nvram_emulation_peek_first_entry(void)
{
    return nvram_emulation_peek_next_entry(NULL);
}


/******************************************************************************
* slist_empty - tests whether a list is empty
* @head: the list to test.
******************************************************************************/
nvram_emulation_entry_t *nvram_emulation_peek_next_entry(nvram_emulation_entry_t *p_buffer)
{
    nvram_emulation_buffer_list_t *p;
    struct slist_node_t *p_node = NULL;
    nvram_emulation_entry_t *p_ret = NULL;
    if(NULL == p_buffer)
    {
        // get the nvram_emulation_buffer_list_t list first entry
        p_node = slist_front(&nvram_emulation_buffer_list.node);
    }
    else
    {
        // get the nvram_emulation_buffer_list_t pointer from the nvram_emulation_entry_t pointer
        p = slist_entry(p_buffer, nvram_emulation_buffer_list_t, entry);
        if(&p->node != slist_tail(&nvram_emulation_buffer_list.node))
        {
            p_node = p->node.next;
        }
    }
    if(p_node)
    {
        // get the nvram_emulation_buffer_list_t pointer from the node pointer
        p = slist_entry(p_node, nvram_emulation_buffer_list_t, node);
        // get the entry pointer
        p_ret = &p->entry;
    }
    return p_ret;
}
