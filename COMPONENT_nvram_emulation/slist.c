/*
 * Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
*
* File Name: slist.c
*
* Abstract:  This file contains utilities of generic single linked list
*
* Functions:
*
*******************************************************************************/
//! \file
//! Abstract:  This file contains utilities of generic single linked list

/*
* overview of the generic single linked list structure:

            +------+
list_head:  | next |--------------------------------------+
            +------+                                      |
                                                            |
                                                            |
                                                            |
                                                            v
        +-----------+       +-----------+           +-----------+
list:   | item1     |       | item2     |           | itemN     |
        |           |       |           |           |           |
        |           |       |           |           |           |
        +-----------+       +-----------+           +-----------+
+----->|   next    |------>|   next    |   ...  -->|   next    |-------+
|      +-----------+       +-----------+           +-----------+       |
|      |           |       |           |           |           |       |
|      |           |       |           |           |           |       |
|      +-----------+       +-----------+           +-----------+       |
|                                                                      |
|                                                                      |
|                                                                      |
+----------------------------------------------------------------------+

The reason that the list_head points to the last item of the list instead
of first item is for better performance to insert an item at the tail.
*
*/
#include "slist.h"

//! slist_add_before - add a new entry.
//! @ new: new entry to be added
//! @ node: the node that will be pointed by new->next.
//! @ head: list head to add it after
//! Insert a new entry in front of existing node
void slist_add_before(struct slist_node_t *_new,
                        struct slist_node_t *node,
                        struct slist_node_t *head)
{
    struct slist_node_t* tmp;
    tmp = head->next;
    while(tmp->next != node)
    {
        tmp = tmp->next;

        // error condition
        if((tmp == head->next) || (!tmp))
        {
            ASSERT(0);
            return;
        }
    }

    tmp->next = _new;
    _new->next = node;
}


//! slist_del - deletes entry from list.
//! @ entry: the element to delete from the list.
//! @ head: list head where the entry is
void slist_del(struct slist_node_t *entry, struct slist_node_t* head)
{
    struct slist_node_t* prev;

    // This function assume the entry is in the list. It's up to caller to
    // guarentee this.

    if (NULL == head->next)
    {
        return;
    }

    for(prev = head->next; prev->next != entry; prev = prev->next)
    {
        if((!prev) || (prev->next == head->next))
        {
            ASSERT(0);
            return;
        }
    }

    if(prev == entry)
    {
        // there is only one item in the list
        head->next = 0;
    }
    else
    {
        if(head->next == entry)
        {
            // This entry is the last item of the list
            head->next = prev;
        }
        prev->next = entry->next;
    }
    entry->next = 0;
}
