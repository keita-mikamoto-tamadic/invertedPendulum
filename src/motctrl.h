#ifndef _MOTCTRL_H_
#define _MOTCTRL_H_

#include "userdefine.h"

/* global variables and function */
typedef struct
{
    float actTorq;

    float actPos_1;
    float actVel_1;

    float actPos_2;
    float actVel_2;
} st_motctrl;
extern st_motctrl stg_motctrl;

extern void MotSetup(void);
extern void MotTorqWrite(float torq, int id);
extern void STSReqPos(int id);

#endif // _MOTCTRL_H_