/**
  ******************************************************************************
  * @file    adv_buff_alloc_tiny.c
  * @author  AMS - RF Application team
  * @brief   Module providing a simplified scheme for buffer allocation for
  *          legacy advertising.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
**/
#include <stdint.h>
#include <stdlib.h>
#include "adv_buff_alloc_tiny.h"

uint8_t buffer[NUM_BUFFERS][31];

/* Variable used to keep which buffer is used and which one is not in use by the link layer. */
uint8_t used_buffers_msk;

#define BUFFER_IN_USE(i)        (used_buffers_msk & (1<<i))
#define SET_BUFFER_IN_USE(i)    (used_buffers_msk |= (1<<i))
#define CLEAR_BUFFER_IN_USE(i)  (used_buffers_msk &= ~(1<<i))

/**
* @brief  Initialize the module for buffer allocation.
* @retval None
*/
void adv_tiny_buff_init(void)
{
  used_buffers_msk = 0;
}

/**
* @brief  Mark a buffer as free.
* @param  buff Pointer to the buffer
* @retval None
*/
void adv_tiny_buff_free(void * p)
{
  for(int i = 0; i < NUM_BUFFERS; i++)
  {
    if(p == buffer[i])
    {
      CLEAR_BUFFER_IN_USE(i);
      return;
    }
  }
}

/**
* @brief  Allocate a buffer of 31 bytes for legacy advertising or scan response data.
* @retval It returns the pointer to the buffer.
*/
void * adv_tiny_buff_alloc(void)
{
  uint8_t i;
    
  for(i = 0; i < NUM_BUFFERS && BUFFER_IN_USE(i) ; i++);
  
  if(i == NUM_BUFFERS)
  {
    // No free buffer found.
    return NULL;
  }
  
  SET_BUFFER_IN_USE(i);
  
  return buffer[i];
}
