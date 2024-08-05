/*******************************************************************************
 * File Name    : motctrl.h
 * Description  : モータ制御の実装
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2024.07.14 1.00 K.Mikamoto 新設
 *****************************************************************************/
#ifndef _MOTCTRL_H_
#define _MOTCTRL_H_

#include "userdefine.h"

/******************************************************************************
 * Macro definitions
 *****************************************************************************/

/*******************************************************************************
 * Typedef definitions
 *****************************************************************************/
typedef struct {
  float act_pos;
  float act_trq;
  float act_vel;
} st_motctrl;

/******************************************************************************
 * Global variables
 *****************************************************************************/
extern st_motctrl stg_motctrl[kMotNum];

/******************************************************************************
 * Global functions
 *****************************************************************************/
extern void MotSetup(MotID_t id);
extern void StartSerial1(void);
extern void MotAllTest(float torq, MotID_t id);
extern void MotTorqWrite(float torq, MotID_t id);
extern void MotPosVelRead(MotID_t id);

#endif  // _MOTCTRL_H_