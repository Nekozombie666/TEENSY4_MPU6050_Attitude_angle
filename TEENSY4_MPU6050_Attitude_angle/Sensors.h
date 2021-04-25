#ifndef SENSORS_H_
#define SENSORS_H_

void ACC_getADC ();     // 加速度
void Gyro_getADC ();    // 角速度
uint8_t Mag_getADC () ; // 磁気

void initSensors ();
void i2c_rep_start (uint8_t address);
void i2c_write (uint8_t data );
void i2c_stop (void);
void i2c_write (uint8_t data );
void i2c_writeReg (uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg (uint8_t add, uint8_t reg);
#ifdef USEWIRELIB
  uint8_t i2c_read(uint8_t ack);
#endif
uint8_t i2c_readAck ();
uint8_t i2c_readNak ();
void i2c_read_reg_to_buf (uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size);



#if defined (MPU6050)
  #define ACC_1G 512
#endif

#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) （accZ比較のcos（theta））
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

// GYRO SCALE: 最後の2ビットを無視し、rad/s に変換します
#if defined (MPU6050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0) //16.4 LSB = 1 deg/s
#endif


#endif /* SENSORS_H_ */
