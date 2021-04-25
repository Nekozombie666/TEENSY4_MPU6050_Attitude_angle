#ifndef MAIN_H_
#define MAIN_H_

#include "types.h"

extern uint32_t currentTime;  // 現在時間
extern uint16_t previousTime; // 前時間
extern uint16_t cycleTime;    // loopサイクル時間
extern uint16_t calibratingA; // ACC較正用
extern uint16_t calibratingG; // Gyro較正用 512回の読み取りからオフセット値を決定する

extern int16_t  i2c_errors_count;

extern global_conf_t global_conf;
/*** types ***/
extern imu_t imu;//types
extern att_t att;

extern conf_t conf;
extern flags_struct_t f;

extern int16_t gyroZero[3];
extern int16_t angle[2];

void annexCode();
#endif /* MAIN_H_ */
