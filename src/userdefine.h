/*******************************************************************************
 * File Name    : userdefine.h
 * Description  : ユーザー定義パラメータ
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2024.04.44 1.00 K.Mikamoto 新設
 *****************************************************************************/

#ifndef _USERDEFINE_H_
#define _USERDEFINE_H_

/******************************************************************************
 * Macro definitions
 *****************************************************************************/
// サーボパラメータ
#define MOT1 1
#define MOT2 2

// 算術定数
#define USER_PI 3.14159265358979323846
#define USER_2PI (2 * USER_PI)

/******************************************************************************
 * Typedef definitions
 *****************************************************************************/
typedef enum { MOTID_0 = 0U, MOTID_1, MOTID_NONE } MotID_t;

/*******************************************************************************
 * Global variables and functions
 *****************************************************************************/
// LQR制御用定数
const float kF1Lqr = -4.207;
const float kF2Lqr = -0.596;
const float kF3Lqr = -0.063;
const float kF4Lqr = -0.082;

// モータ制御用定数
const int kMotNum = 2;
const int kMotResol = 4095;
const int kMotCurGain = 2439;
const float kAngOffset = 0.122;  // 重心ずれの補正 [rad]

#endif  // _USERDEFINE_H_
