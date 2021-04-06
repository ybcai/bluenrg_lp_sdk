#ifndef _ADV_BUFF_ALLOC_H_
#define _ADV_BUFF_ALLOC_H_

/** The number of supported advertising sets. */
#ifndef NUM_ADV_SETS_CONF
#define NUM_ADV_SETS_CONF    2
#endif

void adv_buff_init(void);
uint8_t *adv_buff_alloc(uint8_t handle, uint16_t buffer_len, uint8_t extend, uint16_t *old_buff_len, uint8_t scan_resp);
void adv_buff_free_current(uint8_t handle, uint8_t scan_resp);
void adv_buff_free_next(uint8_t handle, uint8_t scan_resp);
void adv_buff_free_old(uint8_t *buff);
void adv_buff_activate_next(uint8_t handle, uint8_t scan_resp);
void adv_buff_deactivate_current(uint8_t handle, uint8_t scan_resp);

#endif /* _ADV_BUFF_ALLOC_H_ */
