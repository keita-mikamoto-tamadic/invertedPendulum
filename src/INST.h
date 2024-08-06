/*******************************************************************************
 * File Name    : INST.h
 * Description  : Feit シリアルサーボプロトコル コマンド定義
 ******************************************************************************
 * 変更履歴 : DD.MM.YYYY Version Description
 *          : 2021.03.11 1.00 作者 ???
 *****************************************************************************/

#ifndef _INST_H
#define _INST_H

/******************************************************************************
 * Macro definitions
 *****************************************************************************/
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83

// ボーレート定義
#define _1M 0
#define _0_5M 1
#define _250K 2
#define _128K 3
#define _115200 4
#define _76800 5
#define _57600 6
#define _38400 7
#define _19200 8
#define _14400 9
#define _9600 10
#define _4800 11

/******************************************************************************
 * Typedef definitions
 *****************************************************************************/
typedef char s8;
typedef unsigned char u8;
typedef unsigned short u16;
typedef short s16;
typedef unsigned long u32;
typedef long s32;

#endif  // _INST_H