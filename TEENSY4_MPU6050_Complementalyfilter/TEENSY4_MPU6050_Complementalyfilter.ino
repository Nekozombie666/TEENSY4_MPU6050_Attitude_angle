/*
 * Teensy4 MPU6050 Attitude angle with complementalyfilter
   * Teensy4
   * i2c speed 1Mhz
   * loop time 1000 us
   * 
   * Arduino yaw axis drift
   * i2c speed 400kHz
   * loop time 2800 us

 * PINOUT --------------------------------------
   *    Teensy4 ----- MPU6050
   *        Vin ----- VCC  
   *        GND ----- GND
   *      18SDA ----- SDA
   *      19SCL ----- SCL
   *  
   *    Arduino ----- MPU6050
   *         5V ----- VCC
   *        GND ----- GND
   *      A4SDA ----- SDA  
   *      A5SCL ----- SCL

 * 改訂欄 --------------------------------------
   * Sensors.cpp _ Wire.hライブラリを使用選択可能(T4は使用)
   * Sensors.cpp _ 型の追記
   * Sensors.cpp _ 関数i2c_read()の追加
   * IMU.cpp _ 型の追記
   * IMU.cpp _ uint16_t → uint32_t 型の変更(previousT,currentT)
   * IMU.cpp _ signed16 * signed16 高速化使用選択可能(T4は不可)
   * type.h _ __attribute__ ((packed))　を追記
   * def.h _ レジスタ操作による定義を一部Arduino言語に書き換えました。
   * 不要な構文の削除

 */
