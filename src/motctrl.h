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
  float actTorq;

  float actPos_1;
  float actVel_1;

  float actPos_2;
  float actVel_2;

  float test;
  float testtime;
} st_motctrl;

/******************************************************************************
 * Global variables
 *****************************************************************************/
extern st_motctrl stg_motctrl;

/******************************************************************************
 * Global functions
 *****************************************************************************/
extern void MotSetup(void);
extern void MotAllTest(float torq, int id);
extern void MotTorqWrite(float torq, int id);
extern void MotPosVelRead(int id);

#endif  // _MOTCTRL_H_