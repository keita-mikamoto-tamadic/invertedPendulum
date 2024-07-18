#ifndef _MOTCTRL_H_
#define _MOTCTRL_H_

/* global variables and function */
typedef struct
{
    float actTorq;
    float actPos;
} st_motctrl;

extern st_motctrl stg_motctrl;

extern void MotSetup(void);
extern void MotTorqWrite(float torq, int motch);

#endif // _MOTCTRL_H_