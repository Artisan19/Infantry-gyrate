#ifndef REFEREE_DISPATCHTASK_H
#define REFEREE_DISPATCHTASK_H
#include "sys.h"
#include "Referee.h"

void DispchRefereeTask(void *parmas);
uint8_t GetEenmyColor(void);
float GetRealPower(void);
float GetPowerBuffer(void);
Referee_Date *GetRefereeDataPoint(void);
void SendDataToClient(void);

#endif
