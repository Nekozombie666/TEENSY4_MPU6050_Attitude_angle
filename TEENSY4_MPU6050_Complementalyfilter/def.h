#ifndef DEF_H_
#define DEF_H_

/**************************************************************************************/
/* プロセッサ固有の定義 */ // ボード選択によって変更可
/**************************************************************************************/
#if defined(__AVR_ATmega328P__) 
  #define PROMINI
  #define USE
#endif

#if defined(__IMXRT1062__)
  #define TEENSY4
  #define USEWIRELIB //Sensors.cpp で　wire.hを使用する
#endif

/* atmega328P (Promini) **************************************************************/
#if defined(PROMINI)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);              // Arduino pin 13
  #define LEDPIN_TOGGLE              digitalWrite(13,!digitalRead(13)); // switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 digitalWrite(13,HIGH);
  #define LEDPIN_ON                  digitalWrite(13,LOW);
  #define POWERPIN_PINMODE           ;
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;       /*レジスタ操作によるピン割り当て*/ // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #define I2C_SPEED 400000L          // 400kHz fast mode
  #define LOOP_TIME 2800             // 2800us 
#endif
/* IMXRT1062 (Teensy4) **************************************************************/
#if defined(TEENSY4)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_TOGGLE              digitalWrite(13,!digitalRead(13)); // switch LEDPIN state
  #define LEDPIN_OFF                 digitalWrite(13,HIGH);
  #define LEDPIN_ON                  digitalWrite(13,LOW);
  #define POWERPIN_PINMODE           ;
  #define POWERPIN_ON                ;
  #define POWERPIN_OFF               ;
  #define I2C_PULLUPS_ENABLE         I2C_PULLUP_INT
  #define I2C_PULLUPS_DISABLE        I2C_PULLUP_EXT
  #define LOOP_TIME 1000             // 1000us  T4 only 
  #define I2C_SPEED 1000000L         // 1MHz T4 only
#endif

/**************************************************************************************/
/* IMUの向きとセンサーの定義 */
/**************************************************************************************/
#define MPU6050 // acc + gyro

#if defined(MPU6050)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS//Sensors
#endif

/**************************************************************************************/
/* センサータイプの定義 */
/**************************************************************************************/

#if defined(MPU6050)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(MPU6050)
  #define GYRO 1
#else
  #define GYRO 0
#endif


#endif /* DEF_H_ */
