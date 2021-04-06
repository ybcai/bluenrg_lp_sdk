/**
  ******************************************************************************
  * @file    adv_buff_alloc.c
  * @author  AMS - RF Application team
  * @brief   Module providing buffer allocation for advertising data.
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
#include "adv_buff_alloc.h"


struct adv_set_info_s{
  uint8_t  handle;
  uint8_t *old_buff_data;
  uint8_t *curr_buff_data;
  uint8_t *next_buff_data;
  uint16_t next_buff_len;
}adv_buf_info[NUM_ADV_SETS_CONF], scan_resp_buf_info[NUM_ADV_SETS_CONF];

/**
* @brief  Initialize the module for buffer allocation. Mandatory before any use of the module.
* @retval None
*/
void adv_buff_init(void)
{
  uint8_t i;
  
  for(i = 0; i < NUM_ADV_SETS_CONF; i++)
    adv_buf_info[i].handle = 0xFF;
  for(i = 0; i < NUM_ADV_SETS_CONF; i++)
    scan_resp_buf_info[i].handle = 0xFF;
}

/**
* @brief  Retrieve buffer info
* @param  handle Advertising handle
* @param  scan_resp If 0, search for buffer info for advertising data.
                    Otherwise, search for scan response data.
* @retval It returns the pointer to the buffer info structure
*/
static struct adv_set_info_s *search_handle(uint8_t handle, uint8_t scan_resp)
{
  int i;
  struct adv_set_info_s *info;
  
  if(!scan_resp)
    info = adv_buf_info;
  else
    info = scan_resp_buf_info;  
  
  for(i = 0; i < NUM_ADV_SETS_CONF; i++){
    if(info[i].handle == handle)
      return &info[i];
  }
  return NULL;
}

/**
* @brief  It allocates a buffer for advertising or scan response data.
* @param  handle Advertising handle
* @param  buffer_len New length to be allocated
* @param  extend If 0, it allocates a new buffer for the given advertising set, after having been
*                freed any previously allocated buffer.
*                If 1, increase the size of the current block allocated for the advertising set, possibly
*                allocating a new block different from the previous one (in this case with a different address).
* @param[out] old_buff_len Size of the old buffer in case of an extension, otherwise 0.
* @param  scan_resp If 0, allocate a buffer for advertising data, otherwise allocate for scan response data.
* @retval It returns the pointer to the entire buffer.
*/
uint8_t *adv_buff_alloc(uint8_t handle, uint16_t buffer_len, uint8_t extend, uint16_t *old_buff_len, uint8_t scan_resp)
{
  struct adv_set_info_s *info;
  
  *old_buff_len = 0;
  
  info = search_handle(handle, scan_resp);
  if(info == NULL){
    // No existing handle found. Search free locations.
    info = search_handle(0xFF, scan_resp);
    if(info){
      info->handle = handle;
      info->old_buff_data = NULL;
      info->curr_buff_data = NULL;
      info->next_buff_data = NULL;
      info->next_buff_len = 0;
    }
    else {
      // No free locations
      return NULL;
    }
  }
  
  if(!extend){ // New allocation
    
    if(info->old_buff_data) // A buffer which still need to be freed. Do not allow new allocations.
      return NULL;
    
    if(info->next_buff_data) // A buffer has been previously allocated
      free(info->next_buff_data);
    
    if(buffer_len)
      info->next_buff_data = malloc(buffer_len);
    else
      info->next_buff_data = NULL;
    if(info->next_buff_data)
      info->next_buff_len = buffer_len;
    else
      info->next_buff_len = 0;
  }
  else { // Reallocation
    uint8_t *buffer;
    if(!info->next_buff_data) // No buffer previously allocated
      return NULL;
    buffer = realloc(info->next_buff_data, info->next_buff_len + buffer_len);
    if(buffer){
      info->next_buff_data = buffer;
      *old_buff_len = info->next_buff_len;
      info->next_buff_len += buffer_len;
    }
    else {
      free(info->next_buff_data);
      info->next_buff_data = 0;
      info->next_buff_len = 0;
    }
  }
  return info->next_buff_data;  
}

/**
* @brief  Free the buffer that is currently active.
* @param  handle Advertising handle
* @param  scan_resp If 0, free the buffer allocated for advertising data,
*                   otherwise free the one for scan response data.
* @retval None
*/
void adv_buff_free_current(uint8_t handle, uint8_t scan_resp)
{
  struct adv_set_info_s *info;
  
  info = search_handle(handle, scan_resp);  
  if(info == NULL)
    return;
  
  free(info->curr_buff_data);
  info->curr_buff_data = NULL;  
}

/**
* @brief  Free the buffer that is allocated to hold next data
* @param  handle Advertising handle
* @param  scan_resp If 0, free the buffer allocated for advertising data,
*                   otherwise free the one for scan response data.
* @retval None
*/
void adv_buff_free_next(uint8_t handle, uint8_t scan_resp)
{
  struct adv_set_info_s *info;
  
  info = search_handle(handle, scan_resp);  
  if(info == NULL)
    return;
  
  free(info->next_buff_data);
  info->next_buff_data = NULL;  
  info->next_buff_len = 0;
}

/**
* @brief  Free the old buffer, which was waiting to be abandoned by the stack
* @param  buff Pointer to the buffer
* @retval None
*/
void adv_buff_free_old(uint8_t *buff)
{
  uint8_t i, j;
  struct adv_set_info_s * const infos[] = {adv_buf_info, scan_resp_buf_info};
  
  if(buff == NULL)
    return;
  
  for(j = 0; j < 2; j++){
    
    for(i = 0; i < NUM_ADV_SETS_CONF; i++){
      if(infos[j][i].old_buff_data == buff){
        free(buff);
        infos[j][i].old_buff_data = NULL;
        return;
      }
      /* Check also if it has been requested to free the current buffer.
         This may happen if the advertising set is removed. */    
      if(infos[j][i].curr_buff_data == buff){
        /* Free buffers and remove info about this handle, since advertising set
           has been removed */
        free(infos[j][i].curr_buff_data);
        infos[j][i].curr_buff_data = NULL;
        //free(adv_buf_info[i].old_buff_data); This line should not be needed.
        infos[j][i].handle = 0xFF;
        return;
      }    
    }
    
  }
}

/**
* @brief  Function to be called when the new allocated buffer is succesfully passed
*         to the stack to be the new buffer. The new allocated buffer is now
*         considered as the current buffer in use by the stack. If needed, call
*         adv_buff_deactivate_current() before calling this function.
* @param  handle Advertising handle
* @param  scan_resp Set to 0 to reference the buffer allocated for advertising data,
*                   set to 1 for scan response data.
* @retval None
*/
void adv_buff_activate_next(uint8_t handle, uint8_t scan_resp)
{
  struct adv_set_info_s *info;
  
  info = search_handle(handle, scan_resp);  
  if(info == NULL)
    return;
  
  info->curr_buff_data = info->next_buff_data;
  info->next_buff_data = NULL;
  info->next_buff_len = 0;  
}

/**
* @brief  Function to be called after having been informed the stack that the current
          buffer does not have to be used anymore.
* @param  handle Advertising handle
* @param  scan_resp Set to 0 to reference the buffer allocated for advertising data,
*                   set to 1 for scan response data.
* @retval None
*/
void adv_buff_deactivate_current(uint8_t handle, uint8_t scan_resp)
{
  struct adv_set_info_s *info;
  
  info = search_handle(handle, scan_resp);  
  if(info == NULL)
    return;
  
  info->old_buff_data = info->curr_buff_data; // What is in old_buff_data has to be freed.
  info->curr_buff_data = NULL;
}


