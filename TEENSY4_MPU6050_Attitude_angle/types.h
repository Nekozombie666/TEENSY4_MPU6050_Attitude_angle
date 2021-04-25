#ifndef TYPES_H_
#define TYPES_H_

enum rc {
  ROLL,  // FRONT REAR
  PITCH, // LEFT RIGHT
  YAW
};

typedef struct  __attribute__ ((packed)) {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu_t;

typedef struct  __attribute__ ((packed)) {
  int16_t angle[2]; // 0.1度の倍数の絶対角度傾斜    180 deg = 1800
  int16_t heading;             
} att_t;

typedef struct  __attribute__ ((packed)) {
  uint8_t ARMED :1 ;
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
} flags_struct_t;


typedef struct  __attribute__ ((packed)) {
  int16_t  accZero[3];
} global_conf_t;

typedef struct  __attribute__ ((packed)) {
  int16_t angleTrim[2]; 
} conf_t;



#endif /* TYPES_H_ */
