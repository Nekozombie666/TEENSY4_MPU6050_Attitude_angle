#include "Arduino.h"
#include "def.h"
#include "types.h"
#include "Main.h"
#include "IMU.h"
#include "Sensors.h"

#ifdef USEWIRELIB
  #include "Wire.h"
#endif

static void Device_Mag_getADC();
static void Mag_init();
static void ACC_init();

/************************************************************************************************************/
/* board orientation and setup */
/************************************************************************************************************/
//default board orientation
#if !defined (ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#endif
#if !defined (GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
#endif

//MPU6050 MPU9250 Gyro LPF setting
#define GYRO_DLPF_CFG   0 //デフォルト設定LPF 256Hz / 8000Hzサンプル

static uint8_t rawADC[6];

/************************************************************************************************************/
/* I2C general functions */
/************************************************************************************************************/
void i2c_init (void) {
  #ifdef USEWIRELIB
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    i2c_errors_count = 0;
  #else // PROMINI
    #if defined(INTERNAL_I2C_PULLUPS)
      I2C_PULLUPS_ENABLE
    #else
      I2C_PULLUPS_DISABLE
    #endif
    TWSR = 0;                                    // プリスケーラなし=>プリスケーラ= 1
    TWBR = ((F_CPU / 400000) - 16) / 2;          // I2Cクロックレートを400kHzに設定します
    TWCR = 1<<TWEN;                              // twiモジュールを有効にし、割り込みなし
    i2c_errors_count = 0;
  #endif
  
}

void __attribute__ ((noinline)) waitTransmissionI2C(uint8_t twcr) {
  #ifdef USEWIRELIB
    i2c_errors_count++;
  #else // PROMINI
    TWCR = twcr;
    uint8_t count = 255;
    while (!(TWCR & (1<<TWINT))) {
      count--;
      if (count==0) {              //we are in a blocking state => we don't insist
        TWCR = 0;                  //TWINTレジスタを強制的にリセットします
        i2c_errors_count++;
        break;
      }
    }
  #endif
}

void i2c_rep_start (uint8_t address) {
  #ifdef USEWIRELIB
    if (address & 0x01) Wire.requestFrom((uint8_t)(address >>1) ,(uint8_t)10,(uint8_t)false);
    else Wire.beginTransmission((uint8_t) (address>>1));
  #else // PROMINI
    waitTransmissionI2C ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN)); // REPEAT START条件を送信し、送信が完了するまで待ちます
    TWDR = address;                                            // デバイスアドレスを送信する
    waitTransmissionI2C ((1<<TWINT) | (1<<TWEN));              // 送信が完了するまで待つ
  #endif
}
  
  
void i2c_stop (void) {
  #ifdef USEWIRELIB
    Wire.endTransmission(true);
  #else // PROMINI
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  #endif
}

void i2c_write (uint8_t data ) {
  #ifdef USEWIRELIB 
    Wire.write(data);
  #else // PROMINI
    TWDR = data;                                 // 以前にアドレス指定されたデバイスにデータを送信します
    waitTransmissionI2C ((1<<TWINT) | (1<<TWEN));
  #endif
}

#ifdef USEWIRELIB
  uint8_t i2c_read(uint8_t ack) {
    uint8_t r;
    r = Wire.read();
    if (!ack) Wire.endTransmission(ack);
    return r;
  }
#endif

uint8_t i2c_readAck () {
  #ifdef USEWIRELIB 
    return i2c_read(1);
  #else // PROMINI
    waitTransmissionI2C ((1<<TWINT) | (1<<TWEN) | (1<<TWEA));
    return TWDR;
  #endif
  
}

uint8_t i2c_readNak () {
  #ifdef USEWIRELIB 
    return i2c_read(0);
  #else // PROMINI
    waitTransmissionI2C ((1<<TWINT) | (1<<TWEN));
    uint8_t r = TWDR;
    i2c_stop ();
    return r;
  #endif
}

void i2c_read_reg_to_buf (uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size) {
  #ifdef USEWIRELIB 
    uint8_t *b = buf;
    Wire.beginTransmission((uint8_t) add); 
    Wire.write(reg);
    Wire.endTransmission();
    delayMicroseconds(5);
    Wire.beginTransmission((uint8_t) add); 
    if (Wire.requestFrom ((uint8_t)add,(uint8_t)size,(uint8_t)true) == 0) i2c_errors_count++; //hide
      while(Wire.available() && size){    // slave may send less than requested //hide
        size--;   
        *b++ = Wire.read();    // receive a byte as character //hide
      } 
    Wire.endTransmission(true); 
  #else // PROMINI
    i2c_rep_start (add<<1); // I2C書き込み方向
    i2c_write (reg);        // レジスタ選択
    i2c_rep_start ((add<<1) | 1);  // I2C読み取り方向
    uint8_t *b = buf;
    while (--size) *b++ = i2c_readAck(); // 最終バイトを除くすべてを確認する
    *b = i2c_readNak ();
  #endif
  
}


void i2c_getSixRawADC (uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf (add, reg, rawADC, 6);
}

void i2c_writeReg (uint8_t add, uint8_t reg, uint8_t val) {
  #ifdef USEWIRELIB 
    Wire.beginTransmission((uint8_t) add); 
    Wire.write(reg);       
    Wire.write(val);
    Wire.endTransmission(true);
  #else // PROMINI  
    i2c_rep_start (add<<1); // I2C書き込み方向
    i2c_write (reg);        // レジスタ選択
    i2c_write (val);        // レジスタに書き込む値
    i2c_stop();
  #endif  
}

uint8_t i2c_readReg (uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf (add, reg, &val, 1);
  return val;
}

/************************************************************************************************************/
/* GYRO common part */
/************************************************************************************************************/
void GYRO_Common () {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;

  if (calibratingG > 0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingG == 512) { // キャリブレーションの開始時にg [軸]をリセットします
        g[axis] = 0;
      }
      g[axis] += imu.gyroADC[axis]; // 512の読み取り値を合計する
      gyroZero[axis] = g[axis] >> 9;
      if (calibratingG == 1) {
      }
    }
      calibratingG--;
  }
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    // 2つの連続した読み取り値の間の変動を制限します
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800); 
    previousGyroADC[axis] = imu.gyroADC[axis];
  }
}

/************************************************************************************************************/
/* ACC common part */
/************************************************************************************************************/
void ACC_Common () {
  static int32_t a[3];
  if (calibratingA > 0) {
    calibratingA--;
    for (uint8_t axis = 0; axis < 3; axis++) {
      if (calibratingA == 511) a[axis] = 0;     // キャリブレーションの開始時にa [axis]をリセットします
      a[axis] +=imu.accADC[axis];               // 512の読み取り値を合計する
      global_conf.accZero[axis] = a[axis] >> 9; //(calibratingA == 0) が関係する最後の反復のみの平均を計算する
    }
    if (calibratingA == 0) {
      global_conf.accZero[YAW] -= ACC_1G;   // ACC_1GだけZをシフトダウンし、キャリブレーションの終了時にEEPROMに値を保存します
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
    }
  }
  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;
}

/************************************************************************************************************/
/* I2C Gyroscope and Accelerometer MPU6050 */
/************************************************************************************************************/
#if defined (MPU6050)
#if !defined (MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // アドレスピンAD0LOW（GND）、FreeIMU v0.4およびInvenSense評価ボードのデフォルト
  //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

static void Gyro_init () {
  i2c_writeReg (MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(50);
  i2c_writeReg (MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg (MPU6050_ADDRESS, 0x1A, GYRO_DLPF_CFG);    //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg (MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // AUX I2CのI2Cバイパスを有効にします
}

void Gyro_getADC () {
  i2c_getSixRawADC (MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((int16_t)(rawADC[0]<<8) | (int16_t)rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((int16_t)(rawADC[2]<<8) | (int16_t)rawADC[3])>>2 ,
                    ((int16_t)(rawADC[4]<<8) | (int16_t)rawADC[5])>>2 );
  GYRO_Common ();
}

static void ACC_init () {
  i2c_writeReg (MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
}

void ACC_getADC () {
  i2c_getSixRawADC (MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((int16_t)(rawADC[0]<<8) | (int16_t)rawADC[1])>>3 ,
                   ((int16_t)(rawADC[2]<<8) | (int16_t)rawADC[3])>>3 ,
                   ((int16_t)(rawADC[4]<<8) | (int16_t)rawADC[5])>>3 );
    //Serial.print(imu.accADC[ROLL]);Serial.print(F(","));
    //Serial.print(imu.accADC[PITCH]);Serial.print(F(","));
    //Serial.print(imu.accADC[YAW]);
  ACC_Common ();
}
#endif

/************************************************************************************************************/
/* I2C init function */
/************************************************************************************************************/

void initS () {
  i2c_init ();
  if (GYRO)  Gyro_init ();
  if (ACC)   ACC_init ();
}

void initSensors () {
  uint8_t c = 5;
  #if !defined (DISABLE_POWER_PIN)
    POWERPIN_ON;
    delay(200);
  #endif
  while(c) { // i2cエラーなしですべてのセンサーを初期化するために数回試行します。 この段階でI2Cエラーが発生すると、センサーの設定が間違っている可能性があります
    c--;
    initS ();
    if (i2c_errors_count == 0) break; // 初期化中にエラーなし=>初期化OK
  }
}
