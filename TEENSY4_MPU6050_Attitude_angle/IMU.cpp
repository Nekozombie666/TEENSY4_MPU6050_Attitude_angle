#include "Arduino.h"
#include "def.h"
#include "types.h"
#include "Main.h"
#include "IMU.h"
#include "Sensors.h"




void getEstimatedAttitude ();

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  static int16_t gyroADCinter[3];

  uint16_t timeInterleave = 0;
  #if ACC
    ACC_getADC ();
    getEstimatedAttitude ();
  #endif
  #if GYRO
    Gyro_getADC ();
  #endif
  for (axis = 0; axis < 3; axis++) gyroADCinter[axis] =  (int16_t)imu.gyroADC[axis];
  timeInterleave = micros();
  annexCode ();
  uint8_t t = 0;
  while((int16_t)(micros() - timeInterleave) < 650) t=1; //2つの連続した読み取り間の経験的なインターリーブ遅延
  #if GYRO
    Gyro_getADC ();
  #endif
  for (axis = 0; axis < 3; axis++) {
    gyroADCinter[axis] =  imu.gyroADC[axis] + gyroADCinter[axis];
    // 経験 現在と以前の値の加重値を取ります
    imu.gyroData[axis] = (int16_t)((int16_t)gyroADCinter[axis] + (int16_t)gyroADCprevious[axis])/3;
    gyroADCprevious[axis] = (int16_t)gyroADCinter[axis]>>1;
    if (!ACC) imu.accADC[axis]=0;
  }
}

// **************************************************
//「相補フィルタ」に基づく簡易IMU
// http://starlino.com/imu_guide.htmlからヒントを得た
// ziss_dmによる修正：http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
//このプロジェクトでは以下のアイデアが使用されました。
// 1）回転行列：http://ja.wikipedia.org/wiki/Rotation_matrix
// 2）小角近似：http://ja.wikipedia.org/wiki/Small-angle_approximation
// 3）C. atan2（）のヘイスティングス近似
// 4）最適化のコツ：http://www.hackersdelight.org/
//
//現在磁力計は単独で使用されている別のCFを使用しています
//見出しの近似
// **************************************************

//******  advanced users settings *******************
/* ACCのローパスフィルター係数を設定する
    この値を大きくすると、ACCノイズ（GUIで表示）は減少しますが、ACCの遅延時間は増加します
    フィルターを使用したくない場合は、これをコメントしてください。
    単位= 2のn乗 */
// これはALT HOLD計算にも使用されます。変更しないでください
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* ジャイロ/ Acc補完フィルターのジャイロ重量を設定する
    この値を大きくすると、フィルターの出力へのAccの影響が減少および遅延します*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 10 //  that means a CMP_FACTOR of 1024 (2^10)
#endif

/* ジャイロ/磁力計補完フィルターのジャイロ重量を設定する
    この値を増やすと、磁力計がフィルターの出力に与える影響を減らし、遅らせることができます*/
#define GYR_CMPFM_FACTOR 8 // that means a CMP_FACTOR of 256 (2^8)


typedef struct  __attribute__ ((packed)) {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef struct __attribute__ ((packed)) {
  uint16_t XL; int16_t X;
  uint16_t YL; int16_t Y;
  uint16_t ZL; int16_t Z;
} t_int16_t_vector_def;

// note: we use implicit first 16 MSB bits 32 -> 16 cast. ie V32.X>>16 = V16.X
typedef union __attribute__ ((packed)) {
  int32_t A32[3];
  t_int32_t_vector_def V32;
  int16_t A16[6];
  t_int16_t_vector_def V16;
} t_int32_t_vector;

//return angle , unit: 1/10 degree
int16_t _atan2 (int32_t y, int32_t x){
  float z = y;
  int16_t a;
  uint8_t c;
  c = abs(y) < abs(x);
  if ( c ) {z = z / x;} else {z = x / z;}
  a = 2046.43 * (z / (3.5714 +  z * z));
  if ( c ){
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
    a = 900 - a;
    if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x){ //指数関数
  union __attribute__ ((packed)) {  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f1ffff9 - (int32_t)(conv.i >> 1); 
  return conv.f * (1.68191409f - 0.703952253f * x * conv.f * conv.f);
}

// signed16 * signed16
// 22 cycles
// http://mekonik.wordpress.com/2009/03/18/arduino-avr-gcc-multiplication/
#ifdef PROMINI
  #define MultiS16X16to32(longRes, intIn1, intIn2) \
  asm volatile ( \
  "clr r26 \n\t" \
  "mul %A1, %A2 \n\t" \
  "movw %A0, r0 \n\t" \
  "muls %B1, %B2 \n\t" \
  "movw %C0, r0 \n\t" \
  "mulsu %B2, %A1 \n\t" \
  "sbc %D0, r26 \n\t" \
  "add %B0, r0 \n\t" \
  "adc %C0, r1 \n\t" \
  "adc %D0, r26 \n\t" \
  "mulsu %B1, %A2 \n\t" \
  "sbc %D0, r26 \n\t" \
  "add %B0, r0 \n\t" \
  "adc %C0, r1 \n\t" \
  "adc %D0, r26 \n\t" \
  "clr r1 \n\t" \
  : \
  "=&r" (longRes) \
  : \
  "a" (intIn1), \
  "a" (intIn2) \
  : \
  "r26" \
  )
#endif

int32_t  __attribute__ ((noinline)) mul(int16_t a, int16_t b) {
  int32_t r;
  #ifdef PROMINI
    MultiS16X16to32(r, a, b);
  #else
    r = (int32_t)a*b; //without asm requirement
  #endif
  return r;
}

// ジャイロデータに従って、推定角度ベクトルを小さな角度で回転させます。
void rotateV32 (t_int32_t_vector *v,int16_t* delta) {
  int16_t X = v->V16.X;
  int16_t Y = v->V16.Y;
  int16_t Z = v->V16.Z;

  v->V32.Z -=  mul(delta[ROLL]  ,  X)  + mul(delta[PITCH] , Y);
  v->V32.X +=  mul(delta[ROLL]  ,  Z)  - mul(delta[YAW]   , Y);
  v->V32.Y +=  mul(delta[PITCH] ,  Z)  + mul(delta[YAW]   , X);
}

static int16_t accZ=0;

void getEstimatedAttitude () {
  uint8_t axis;
  int32_t accMag = 0;
  float scale;
  int16_t deltaGyroAngle16[3];
  static t_int32_t_vector EstG = {0,0,(int32_t)ACC_1G<<16};
  static t_int32_t_vector EstM = {0,(int32_t)1<<24,0};
  static uint32_t LPFAcc[3];
  float invG; // 1/|G|
  static int16_t accZoffset = 0;
  int32_t accZ_tmp=0;
  static uint32_t previousT;
  uint32_t currentT = micros();

//単位：ビットあたりのラジアン、さらに乗算するために2 ^ 16でスケーリング
//デルタタイムが3000 usで、ジャイロスケールがほとんどのジャイロで、スケール= 1より少し小さい
  scale = (currentT - previousT) * (GYRO_SCALE * 65536);
  previousT = currentT;

  // 初期化
  for (axis = 0; axis < 3; axis++) {
    // LPF_FACTORが15未満である限り有効
    imu.accSmooth[axis]  = (int16_t)LPFAcc[axis]>>ACC_LPF_FACTOR;
    LPFAcc[axis] += imu.accADC[axis] - imu.accSmooth[axis];
    // 後でaccベクトルの大きさを計算するために使用されます
    accMag += mul(imu.accSmooth[axis] , imu.accSmooth[axis]);
//単位：2 ^ 16でスケーリングされたラジアン
// imu.gyroADC [axis]の長さは14ビット、スケール係数により、deltaGyroAngle16 [axis]の長さは14ビットのままです
    deltaGyroAngle16[axis] = (int16_t)imu.gyroADC[axis] * scale;
  }
//中間の32ビットベクトルをラジアンベクトル（deltaGyroAngle16）で回転させ、2 ^ 16でスケーリングします
//ただし、32ビットベクトルの最初の16 MSBのみが結果の計算に使用されます
// 16 LSBは相補フィルタ部分にのみ使用されるため、この近似値を使用しても問題ありません
  rotateV32(&EstG,deltaGyroAngle16);
  rotateV32(&EstM,deltaGyroAngle16);

//補完フィルターを適用（ジャイロドリフト補正）
//加速度の大きさが> 1.15Gまたは<0.85Gで、ACCが限界範囲外の場合=>角度推定における加速度計の効果を無効にします。
//これを行うには、EstVがすでにジャイロによって回転しているため、フィルターをスキップします
  for (axis = 0; axis < 3; axis++) {
    if ( (int16_t)(0.85*ACC_1G*ACC_1G/256) < (int16_t)(int32_t)(accMag>>8) && (int16_t)(int32_t)(accMag>>8) < (int16_t)(1.15*ACC_1G*ACC_1G/256) )
      EstG.A32[axis] += (int32_t)(int32_t)(imu.accSmooth[axis] - EstG.A16[2*axis+1])<<(16-GYR_CMPF_FACTOR);
    accZ_tmp += mul(imu.accSmooth[axis] , EstG.A16[2*axis+1]);
  }
  
  if (EstG.V16.Z > ACCZ_25deg)
    f.SMALL_ANGLES_25 = 1;
  else
    f.SMALL_ANGLES_25 = 0;

  // 推定ベクトルの姿勢
  int32_t sqGX_sqGZ = mul(EstG.V16.X,EstG.V16.X) + mul(EstG.V16.Z,EstG.V16.Z);
  invG = InvSqrt(sqGX_sqGZ + mul(EstG.V16.Y,EstG.V16.Y));

  att.angle[ROLL]  = _atan2(EstG.V16.X , EstG.V16.Z);
  att.angle[PITCH] = _atan2(EstG.V16.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);

  //2番目の用語に注意してください。数学的にはオーバーフローのリスクがあります（16 * 16 * 16 = 48ビット）。 実際の値ではnullとみなされる
  att.heading = _atan2(
    mul(EstM.V16.Z , EstG.V16.X) - mul(EstM.V16.X , EstG.V16.Z),
    (EstM.V16.Y * sqGX_sqGZ  - (mul(EstM.V16.X , EstG.V16.X) + mul(EstM.V16.Z , EstG.V16.Z)) * EstG.V16.Y)*invG );
  att.heading /= 10;

// ACCベクトルをグローバルZに投影し、1Gをサブストラクチャー
//数学：accZ = A * G / | G | -1G
  accZ = accZ_tmp *  invG;
  if (!f.ARMED) {
    accZoffset -= (int16_t)accZoffset>>3;
    accZoffset += accZ;
  }  
  accZ -= (int16_t)accZoffset>>3;
}
