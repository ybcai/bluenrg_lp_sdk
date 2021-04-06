#ifndef _PROCEDURES_H_
#define _PROCEDURES_H_

extern char name[];

tBleStatus ConfigureWhiteList(void);
tBleStatus ConfigureAdvertising(uint8_t filter_policy);
tBleStatus StartAdvertising(void);
tBleStatus StopAdvertising(void);

void StartGeneralConnectionEstablishment(void);
tBleStatus StartAutoConnection(void);
void StopScan(void);

void BlacklistHit(uint8_t address_type, uint8_t address[]);
void BlacklistReset(void);


#endif /* _PROCEDURES_H_ */
