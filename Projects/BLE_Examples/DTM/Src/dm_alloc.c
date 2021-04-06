/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : dm_alloc.c
* Author             : SRA - BLE stack team
* Description        : Dinamic Memory Allocator
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/******************************************************************************
 * INCLUDE HEADER FILES
 *****************************************************************************/
#include "bluenrg_lp_api.h"
#include "dm_alloc.h"
#include <string.h>
/******************************************************************************
 * LOCAL MACROS
 *****************************************************************************/
#define ALIGN_UPTO_32BITS(VAL)          (((((unsigned int)(VAL)) - 1U) | (sizeof(uint32_t) - 1U)) + 1U)
#define DM_SLICE_THRESHOLD              (3 * sizeof(db_alloc_header_t))
/*#define DM_DEBUG                        (1) */
#if defined(DM_DEBUG)
#define DM_DEBUG_STAMP                  (0xBEAF)
#endif
/******************************************************************************
 * LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *****************************************************************************/
typedef struct db_alloc_header_s {
    uint16_t buffer_size;
    uint16_t flags;
    uint32_t buffer_a[];
} db_alloc_header_t;

typedef struct db_free_header_s {
    uint16_t buffer_size;
#if defined(DM_DEBUG)
    uint16_t flags;
#endif
    struct db_free_header_s *next;
    uint32_t buffer_a[];
} dm_free_header_t;

typedef struct dm_ctx_s {
#if defined(DM_DEBUG)
    uint16_t alloc_size;
    uint16_t alloc_max_size;
#endif
    dm_free_header_t *head;
    uint32_t *alloc_space_p;
} dm_ctx_t;

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 *****************************************************************************/
/******************************************************************************
 * Local Variables
 *****************************************************************************/
static dm_ctx_t dm_ctx;

void dm_init(uint16_t buffer_size, uint8_t *buffer_p)
{
    dm_ctx.alloc_space_p = (void *)buffer_p;
    dm_ctx.head = (dm_free_header_t *)buffer_p;
    dm_ctx.head->buffer_size = buffer_size;
#if defined(DM_DEBUG)
    dm_ctx.head->flags = DM_DEBUG_STAMP;
    dm_ctx.alloc_size = 0U;
    dm_ctx.alloc_max_size = 0U;
#endif
    dm_ctx.head->next = NULL;
}

void db_extract_from_free_list(dm_free_header_t *entry_p)
{
    dm_free_header_t *e_p;

#if defined(DM_DEBUG)
    if (entry_p->flags != DM_DEBUG_STAMP)
    {
        while (1)
            ;
    }
#endif
    if (entry_p == dm_ctx.head)
    {
        dm_ctx.head = entry_p->next;
    }
    else
    {
        e_p = dm_ctx.head;
        while (e_p != NULL)
        {
            if (e_p->next == entry_p)
            {
                e_p->next = entry_p->next;
                break;
            }
            e_p = e_p->next;
        }
    }
}

void db_add_to_free_list(dm_free_header_t *free_entry_p)
{
    dm_free_header_t *prev_p;

    if (free_entry_p != NULL)
    {
#if defined(DM_DEBUG)
        if (free_entry_p->flags != DM_DEBUG_STAMP)
        {
            while (1)
                ;
        }
#endif
        prev_p = NULL;
        if (dm_ctx.head == NULL)
        {
            /**
             * The free list is empty. Assign the new entry to the list.
             */
            dm_ctx.head = free_entry_p;
        }
        else
        {
            if ((uintptr_t)free_entry_p < (uintptr_t)dm_ctx.head)
            {
                /**
                 * Insert the new element at the head of the list.
                 */
                free_entry_p->next = dm_ctx.head;
                dm_ctx.head = free_entry_p;
            }
            else
            {
                prev_p = dm_ctx.head;
                while (prev_p->next != NULL)
                {
                    /**
                     * The free list is ordered by index (address) then search
                     * into the list to find the previous node.
                     */
                    if ((uintptr_t)prev_p->next > (uintptr_t)free_entry_p)
                    {
                        break;
                    }
                    prev_p = prev_p->next;
                }

                /**
                 * Insert the new element.
                 */
                free_entry_p->next = prev_p->next;
                prev_p->next = free_entry_p;

                /**
                 * Try to coalesce the new free entry with the previous.
                 */
                if (((uintptr_t)prev_p + prev_p->buffer_size) ==
                    (uintptr_t)free_entry_p)
                {
                    prev_p->next = free_entry_p->next;
                    prev_p->buffer_size += free_entry_p->buffer_size;
                }
            }

            /**
             * Try to coalesce the new free entry with the next.
             */
            if (((uintptr_t)free_entry_p + free_entry_p->buffer_size) ==
                (uintptr_t)free_entry_p->next)
            {
                dm_free_header_t *ne_p;

                ne_p = free_entry_p->next;
                free_entry_p->next = ne_p->next;
                free_entry_p->buffer_size += ne_p->buffer_size;
            }
        }
    }
}

uint32_t *dm_alloc(uint16_t size)
{
    uint16_t slice_size, alloc_size;
    dm_free_header_t *entry_p, *best_entry_p;

    best_entry_p = NULL;
    entry_p = dm_ctx.head;
    alloc_size = (uint16_t)(ALIGN_UPTO_32BITS(size) + sizeof(db_alloc_header_t));
#if defined(DM_DEBUG)
    if (entry_p != NULL)
    {
        if (entry_p->flags != DM_DEBUG_STAMP)
        {
            while (1)
                ;
        }
    }
#endif
    while (entry_p != NULL)
    {
        /**
         * Best fit strategy: search for the entry that has the size closer to
         * the requested value.
         */
        if (entry_p->buffer_size >= alloc_size)
        {
            if ((best_entry_p == NULL) ||
                ((best_entry_p != NULL) &&
                 (best_entry_p->buffer_size > entry_p->buffer_size)))
            {
                best_entry_p = entry_p;
            }
        }
        entry_p = entry_p->next;
    }

    if (best_entry_p != NULL)
    {
        db_alloc_header_t *alloc_entry_p;

        /**
         * Detach entry by free list.
         */
        db_extract_from_free_list(best_entry_p);

        /**
         * If the extracted entry has a size "much" greater then the
         * requested one then slice it releasing the not requested space.
         */
        slice_size = best_entry_p->buffer_size - alloc_size;
        if (slice_size > DM_SLICE_THRESHOLD)
        {
            dm_free_header_t *slice_p;

            slice_p = (dm_free_header_t *)&best_entry_p->buffer_a[(alloc_size -
            sizeof(dm_free_header_t)) >> 2];
            slice_p->buffer_size = slice_size;
#if defined(DM_DEBUG)
            slice_p->flags = DM_DEBUG_STAMP;
#endif
            slice_p->next = NULL;
            db_add_to_free_list(slice_p);
            best_entry_p->buffer_size = alloc_size;
        }
        alloc_entry_p = (db_alloc_header_t *)best_entry_p;
#if defined(DM_DEBUG)
        if (alloc_entry_p->flags != DM_DEBUG_STAMP)
        {
            while (1)
                ;
        }
        dm_ctx.alloc_size += alloc_size + sizeof(uint32_t);
        if (dm_ctx.alloc_size > dm_ctx.alloc_max_size)
        {
            dm_ctx.alloc_max_size = dm_ctx.alloc_size;
        }
#endif

        return alloc_entry_p->buffer_a;
    }

    return NULL;
}

void dm_free(uint32_t *buffer_p)
{
    dm_free_header_t *free_entry_p;

    free_entry_p = (dm_free_header_t *)(--buffer_p);
    free_entry_p->next = NULL;
#if defined(DM_DEBUG)
    dm_ctx.alloc_size -= free_entry_p->buffer_size;
#endif
    db_add_to_free_list(free_entry_p);
}

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
