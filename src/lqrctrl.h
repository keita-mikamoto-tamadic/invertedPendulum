#ifndef _LQRCTRL_H_
#define _LQRCTRL_H_

/* global variables and function */
typedef struct
{
    float refTorq;
} st_lqr;

extern st_lqr stg_lqr;

extern void LQRcontrol(float pitch);

#endif // _LQRCTRL_H_