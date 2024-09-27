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

#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_rtos.h"
#include "wiced_transport.h"
#include "nvram_emulation_mem.h"
#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

#if NVRAM_EMULATION_HCI

/*
 * nvram_emulation_transport_status_handler
 */
wiced_result_t nvram_emulation_transport_status_handler( wiced_transport_type_t type )
{
    return wiced_transport_send_data(HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0);
}

static wiced_result_t nvram_emulation_transport_rx_data_handler_push_nvram_data(uint8_t *p_data, uint32_t data_len)
{
    uint16_t vs_id;
    uint32_t payload_len;
    wiced_result_t status;

    /* Check parameter. */
    if ((p_data == NULL) ||
        (data_len == 0))
    {
        return WICED_BADARG;
    }

    /* Parse information. */
    STREAM_TO_UINT16(vs_id, p_data);
    payload_len = data_len - sizeof(vs_id);
    WICED_BT_TRACE("[%s] host data restore nvram_emulation_write vs_id 0x%x\n", __FUNCTION__, vs_id);
    nvram_emulation_write(vs_id, payload_len, p_data, &status);
    return status;
}

/*
 * nvram_emulation_transport_rx_data_handler
 */
uint32_t nvram_emulation_transport_rx_data_handler(uint8_t *p_buffer, uint32_t length)
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint32_t result = HCI_CONTROL_STATUS_FAILED;

    /* Check parameter. */
    if (p_buffer == NULL)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    // Expected minimum 4 byte as the wiced header
    if (length < (sizeof(opcode) + sizeof(payload_len)))
    {
        wiced_transport_free_buffer(p_buffer);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    switch(opcode)
    {
    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        if(nvram_emulation_transport_rx_data_handler_push_nvram_data(p_data, payload_len) != WICED_SUCCESS)
        {
            WICED_BT_TRACE("[%s] host push nvram data failed, opcode 0x%x\n", __FUNCTION__, opcode);
        }
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer(p_buffer);
        result = HCI_CONTROL_STATUS_SUCCESS;
        break;
    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        nvram_emulation_delete( p_data[0] | ( p_data[1] << 8 ), (wiced_result_t *)&result );
        WICED_BT_TRACE( "[%s] NVRAM delete: %d, result 0x%x\n", __FUNCTION__, p_data[0] | ( p_data[1] << 8 ), result );
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer(p_buffer);
        result = HCI_CONTROL_STATUS_SUCCESS;
        break;
    default:
        break;
    }

    return result;
}

#endif // NVRAM_ENULATION_HOST_BACKUP
/**
 * find_entry_with_vs_id
 *
 * Searches nvram entries for matching vs_id
 *
 * @return pointer to entry or NULL if no vs_id match found
 */
nvram_emulation_entry_t *find_entry_with_vs_id(uint16_t vs_id)
{
    nvram_emulation_entry_t *p_entry = nvram_emulation_peek_first_entry();
    while(p_entry)
    {
        if(vs_id == p_entry->vs_id)
        {
            break;
        }
        p_entry = nvram_emulation_peek_next_entry(p_entry);
    }
    return p_entry;
}

/**
 * nvram_emulation_read
 *
 * Reads the data from NVRAM
 *
 * @param[in] vs_id         : Volatile Section Identifier. Application can use the VS IDs from
 *                            WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param[in] data_length   : Length of the data to be read from NVRAM
 * @param[out] p_data       : Pointer to the buffer to which data will be copied
 * @param[out] p_status     : Pointer to location where status of the call is returned
 *
 * @return length of data that is read
 */
uint16_t nvram_emulation_read(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status)
{
    nvram_emulation_entry_t *p_entry;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return 0;
    }

    *p_status = WICED_BADARG;

    if ((data_length == 0) ||
        (p_data == NULL))
    {
        return 0;
    }

    *p_status = WICED_ERROR;
    if ((vs_id < WICED_NVRAM_VSID_START) ||
        (vs_id > WICED_NVRAM_VSID_END))
    {
        return 0;
    }

    /* Check if the target vs_id exists. */
    p_entry = find_entry_with_vs_id(vs_id);
    if(p_entry != NULL)
    {
        /* Check the data length. */
        if (data_length < p_entry->length)
        {
            return 0;
        }
        memcpy((void *) p_data, (void *) &p_entry->p_data[0], p_entry->length);
    }

    if (p_entry == NULL)
    {
        return 0;
    }

    *p_status = WICED_SUCCESS;
    return p_entry->length;
}

/**
 * nvram_emulation_write
 *
 * Reads the data to NVRAM
 *
 * @param[in] vs_id         : Volatile Section Identifier. Application can use the VS IDs from
 *                            WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param[in] data_length   : Length of the data to be written to the NVRAM
 * @param[in] p_data        : Pointer to the data to be written to the NVRAM
 * @param[out] p_status     : Pointer to location where status of the call is returned
 *
 * @return number of bytes written, 0 on error
 */
uint16_t nvram_emulation_write(uint16_t vs_id, uint16_t data_length, uint8_t *p_data, wiced_result_t *p_status)
{
    nvram_emulation_entry_t *p_entry;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return 0;
    }

    *p_status = WICED_BADARG;

    if ((data_length == 0) ||
        (p_data == NULL))
    {
        return 0;
    }

    *p_status = WICED_ERROR;
    if ((vs_id < WICED_NVRAM_VSID_START) ||
        (vs_id > WICED_NVRAM_VSID_END))
    {
        return 0;
    }

    /* Check if the target vs_id exists. */
    p_entry = find_entry_with_vs_id(vs_id);
    if (p_entry != NULL)
    {
       /* Check the data length. */
        if (data_length != p_entry->length)
        {
            /* Delete this entry. */
            nvram_emulation_delete(vs_id, p_status);

            /* Add a new entry. */
            return nvram_emulation_write(vs_id, data_length, p_data, p_status);
        }

        /* Check if the data shall be updated. */
        if (memcmp((void *) p_data,
                    (void *) &p_entry->p_data[0],
                    data_length) == 0)
        {
            *p_status = WICED_SUCCESS;
            return data_length;
        }

        /* Update data content. */
        memcpy((void *) &p_entry->p_data[0], (void *) p_data, data_length);

#if NVRAM_EMULATION_HCI
        /* Inform Host device. */
        WICED_BT_TRACE("[%s] inform host nvram update vs_id 0x%x len %d\n", __FUNCTION__, vs_id, data_length);
        *p_status = wiced_transport_send_data(HCI_CONTROL_EVENT_NVRAM_DATA,
                                        (uint8_t *)&p_entry->vs_id,
                                        sizeof(vs_id) + data_length);
        if (*p_status != WICED_SUCCESS)
        {
            /* fail, delete entry due to unsync */
            WICED_BT_TRACE("wiced_transport_send_data update failed, deleting entry\n");
            nvram_emulation_delete(vs_id, p_status);
            return 0;
        }
#endif
        *p_status = WICED_SUCCESS;
        return p_entry->length;
    }

    /* Acquire memory. */
    p_entry = (nvram_emulation_entry_t *) nvram_emulation_entry_allocate(data_length);
    if (p_entry == NULL)
    {
        *p_status = WICED_NO_MEMORY;
        return 0;
    }

    /* Write information. */
    p_entry->vs_id = vs_id;
    p_entry->length = data_length;
    memcpy((void *) &p_entry->p_data[0], (void *) p_data, data_length);
#if NVRAM_EMULATION_HCI
    /* Inform Host device. */
    WICED_BT_TRACE("[%s] inform host nvram update vs_id 0x%x len %d\n", __FUNCTION__, vs_id, data_length);
    *p_status = wiced_transport_send_data(HCI_CONTROL_EVENT_NVRAM_DATA,
                                    (uint8_t *)&p_entry->vs_id,
                                    sizeof(vs_id) + data_length);
    if (*p_status != WICED_SUCCESS)
    {
        nvram_emulation_delete(vs_id, p_status);
        *p_status = WICED_NO_MEMORY;
        return 0;
    }
#endif
    *p_status = WICED_SUCCESS;
    return p_entry->length;
}

/**
 * nvram_emulation_delete
 *
 * Deletes data from NVRAM at specified VS id
 *
 * @param vs_id     : Volatile Section Identifier. Application can use the VS IDs from
 *                    WICED_NVRAM_VSID_START to WICED_NVRAM_VSID_END
 * @param p_status  : Pointer to location where status of the call is returned
 */
void nvram_emulation_delete(uint16_t vs_id, wiced_result_t *p_status)
{
    nvram_emulation_entry_t *p_entry;

    /* Check parameter. */
    if (p_status == NULL)
    {
        return;
    }

    *p_status = WICED_ERROR;
    if ((vs_id < WICED_NVRAM_VSID_START) ||
        (vs_id > WICED_NVRAM_VSID_END))
    {
        return;
    }

    p_entry = find_entry_with_vs_id(vs_id);
    if(p_entry != NULL)
    {
        nvram_emulation_entry_free((void *) p_entry);
        *p_status = WICED_SUCCESS;
    }
}
