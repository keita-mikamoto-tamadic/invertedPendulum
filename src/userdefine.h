#ifndef _USERDEFINE_H_
#define _USERDEFINE_H_

/* servo parameter */
#define MOT1 1
#define MOT2 2
#define MOTRESOL 4095

/* robot parameter */
#define MOTNUM 2

#define F1LQR -4.207
#define F2LQR -0.596
#define F3LQR -0.063
#define F4LQR -0.082

#define MOTCURGAIN 2439

#define ANGOFFS 0.122 /* 重心ずれの補正[rad] */

#define USER_PI (3.14159265358979323846f)
#define USER_2PI (USER_PI * 2.0f)

#endif  //_USERDEFINE_H_