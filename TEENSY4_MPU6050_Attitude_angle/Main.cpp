/***** Main Program ******/
//#include <avr/io.h> // AVRマイコンIOポート操作レジスタ操作用
#include "Arduino.h"
#include "def.h"
#include "types.h"
#include "Main.h"
#include "IMU.h"
#include "Sensors.h"

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // これは完全なループを達成するためのマイクロ秒単位の数値であり、少し異なる場合があり、PIDループで考慮されます
uint16_t calibratingA = 0;  // キャリブレーションはメインループで行われます。 キャリブレーションは各サイクルで0に減少し、通常モードに入ります。
uint16_t calibratingG;

/****************/
/* gyro+acc IMU */
/****************/
#if defined(TEENSY4)
  int16_t gyroZero[3] = {0,0,0};
#endif 

imu_t imu;
att_t att;

flags_struct_t f;
int16_t  i2c_errors_count = 0;

/************************/
/* EEPROMレイアウトの定義 */
/************************/
global_conf_t global_conf;
conf_t conf;

/********************************/
/* Sensors calibration function */
/********************************/
void annexCode () { //このコードは各ループで実行され、650マイクロ秒未満続くと制御ループに干渉しません
  static uint32_t calibratedAccTime;

  if ( (calibratingA > 0 && ACC ) || (calibratingG > 0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // ドローンはACCを使用しており、キャリブレーションされていないか、傾斜しすぎています
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }
}

/*********************************************************************************************************************/
/*************************************************** main setup ******************************************************/
/*********************************************************************************************************************/
void setup () {
  Serial.begin(115200);  // シリアルインターフェイスの速度
  
  initSensors ();        //センサの初期化
  
  previousTime = micros(); 
  calibratingG = 512;    // キャリブレーションの開始時にg [軸]をリセットします

  f.SMALL_ANGLES_25 = 1; // ジャイロのみのconfにとって重要
}/**** setup end ****/

/*********************************************************************************************************************/
/*************************************************** main loop *******************************************************/
/*********************************************************************************************************************/
void loop () {
/**** サイクル時間管理 ****/
  while(1) {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    #if defined(LOOP_TIME)
      if (cycleTime >= LOOP_TIME) break;
    #else
      break;  
    #endif
  }

  previousTime = currentTime;

/* tab function */
  computeIMU ();//姿勢角の算出,IMUtab
 
/* Serial Monitor */
//センサデータ等の確認を行えます．
// Serial.printはloop速度に依存するので注意
  Serial.print(F("DEL:"));//DateGUI
  //Serial.print(F("#ACC:"));//Accelerometer angle
  //Serial.print(imu.accADC[ROLL]);Serial.print(F(","));
  //Serial.print(imu.accADC[PITCH]);Serial.print(F(","));
  //Serial.print(imu.accADC[YAW]);
  //Serial.print(F("#GYR:"));//Gyroscope angle
  //Serial.print(imu.gyroADC[ROLL]);Serial.print(F(","));
  //Serial.print(imu.gyroADC[PITCH]); Serial.print(F(","));
  //Serial.print(imu.gyroADC[YAW]);
  Serial.print(F("#FIL:")); //Filtered angle
  Serial.print(att.angle[ROLL]/10);Serial.print(F(","));//フラッシュメモリの文字列もF()で囲むことで，Serial.print()を使用して出力することができます．
  Serial.print(att.angle[PITCH]/10);Serial.print(F(","));
  Serial.print(att.heading);
  //Serial.print(F(","));Serial.print(cycleTime);
  Serial.println(F("")); //改行
}/**** loop end ****/
