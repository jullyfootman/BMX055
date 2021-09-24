#include <Arduino.h>
#include <Wire.h>

//JP1,JP2,JP3 = Open
#define ADD_ACC 0x19 //加速度センサのI2Cアドレス
#define ADD_YAW 0x69 //ジャイロセンサのI2Cアドレス
#define ADD_MAG 0x13 //磁気センサのI2Cアドレス
#define SDA_pin 26
#define SCL_pin 25
#define DATA_NUM 6// 読み取りデータ数　共通
#define SAMPLING_TIME 100 //読み取り周期 ms
#define ACC_LSB 0.0098// 加速度のLSB
#define YAW_LSB 0.0038// ヨーのLSB


//加速度
float f_xAcc, f_yAcc, f_zAcc;
int16_t s16_xAcc, s16_yAcc, s16_zAcc;
//ヨー
float f_xYaw, f_yYaw, f_zYaw;
int16_t s16_xYaw, s16_yYaw, s16_zYaw;
// 磁気
int16_t xMag, yMag, zMag;
int16_t s16_xMag, s16_yMag, s16_zMag;
//補正値
float f_xAcc_comp, f_yAcc_comp, f_zAcc_comp;
float f_xYaw_comp, f_yYaw_comp, f_zYaw_comp;
int16_t s16_xMag_comp, s16_yMag_comp, s16_zMag_comp;

// レジスタへの書き込み
void setRegister(int8_t i2cAdd, int8_t regAdd, int8_t value)
{
  Wire.beginTransmission(i2cAdd);
  Wire.write(regAdd); // register adrress
  Wire.write(value);  // value
  Wire.endTransmission();
  delay(100);
}

void BMX055_Init()
{
  setRegister(ADD_ACC, 0x0F, 0x03); // Range = +/- 2g
  setRegister(ADD_ACC, 0x10, 0x08); // Bandwidth = 7.81 Hz
  setRegister(ADD_ACC, 0x11, 0x00); // Normal mode, Sleep duration = 0.5ms
  setRegister(ADD_YAW, 0x0F, 0x04); // Full scale = +/- 125 degree/s
  setRegister(ADD_YAW, 0x10, 0x07); // ODR = 100 Hz
  setRegister(ADD_YAW, 0x11, 0x00); // Normal mode, Sleep duration = 2ms
  setRegister(ADD_MAG, 0x4B, 0x83); // Soft reset
  setRegister(ADD_MAG, 0x4B, 0x01); // Soft reset
  setRegister(ADD_MAG, 0x4C, 0x00); // Normal Mode, ODR = 10 Hz
  setRegister(ADD_MAG, 0x4E, 0x84); // X, Y, Z-Axis enabled
  setRegister(ADD_MAG, 0x51, 0x04); // No. of Repetitions for X-Y Axis = 9
  setRegister(ADD_MAG, 0x52, 0x16); // No. of Repetitions for Z-Axis = 15
}
//=====================================================================================//
void readAcc()
{
  uint8_t data[DATA_NUM];
  Wire.beginTransmission(ADD_ACC);
  Wire.write(0x02); // Select data register
  Wire.endTransmission();
  Wire.requestFrom(ADD_ACC, DATA_NUM);
  if (Wire.available() == DATA_NUM)
  {
    for (int i = 0; i < DATA_NUM; i++)
      data[i] = Wire.read();
  }
  uint16_t u2_temp = 0;
  s16_xAcc = 0;
  u2_temp = ((data[1] << 8) + data[0]) >> 4; //12bit
  //負数（2の補数）処理　bitを調整して(2の補数処理されているはずなので)int16-tに直接入れる
  if (u2_temp >> 11)                          // 先頭bitが1であれば
    s16_xAcc = u2_temp | 0b1111000000000000; // 上位を1で埋める 16bitの2の補数による負数
  else
    s16_xAcc = u2_temp;

  u2_temp = 0;
  s16_yAcc = 0;
  u2_temp = ((data[3] << 8) + data[2]) >> 4; //12bit
  if (u2_temp >> 11)
    s16_yAcc = u2_temp | 0b1111000000000000;
  else
    s16_yAcc = u2_temp;

  u2_temp = 0;
  s16_zAcc = 0;
  u2_temp = ((data[5] << 8) + data[4]) >> 4; //12bit
  if (u2_temp >> 11)
    s16_zAcc = u2_temp | 0b1111000000000000;
  else
    s16_zAcc = u2_temp;

  f_xAcc = s16_xAcc * ACC_LSB + f_xAcc_comp; // renge +-2g
  f_yAcc = s16_yAcc * ACC_LSB + f_yAcc_comp; // renge +-2g
  f_zAcc = s16_zAcc * ACC_LSB + f_zAcc_comp; // renge +-2g
}
//=====================================================================================//
void readYaw()
{
  uint8_t data[DATA_NUM];
  Wire.beginTransmission(ADD_YAW);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(ADD_YAW, DATA_NUM);
  if (Wire.available() == DATA_NUM)
  {
    for (int i = 0; i < DATA_NUM; i++)
      data[i] = Wire.read();
  }
  s16_xYaw = (data[1] << 8) + data[0];//16bit
  s16_yYaw = (data[3] << 8) + data[2];//16bit
  s16_zYaw = (data[5] << 8) + data[4];//16bit

  f_xYaw = s16_xYaw * YAW_LSB;
  f_yYaw = s16_yYaw * YAW_LSB;
  f_zYaw = s16_zYaw * YAW_LSB;

  f_xYaw += f_xYaw_comp; //オフセット補正
  f_yYaw += f_yYaw_comp; //オフセット補正
  f_zYaw += f_zYaw_comp; //オフセット補正
}

//=====================================================================================//
void readMag()
{
  short int data[DATA_NUM];
  Wire.beginTransmission(ADD_MAG);
  Wire.write(0x42); // Select data register
  Wire.endTransmission();
  Wire.requestFrom(ADD_MAG, DATA_NUM);
  if (Wire.available() == DATA_NUM)
  {
    for (int i = 0; i < DATA_NUM; i++){
      data[i] = Wire.read();}
  }
  uint16_t u2_temp = 0;
  s16_xMag = 0;
  u2_temp = ((data[1] << 8) + data[0]) >> 3; //13bit

  if (u2_temp >> 12)                         // 先頭bitが1であれば
    s16_xMag = u2_temp | 0b1110000000000000; // 上位を1で埋める 16bitの2の補数による負数
  else
    s16_xMag = u2_temp;

  u2_temp = 0;
  s16_yMag = 0;
  u2_temp = ((data[3] << 8) + data[2]) >> 3; //13bit
  if (u2_temp >> 12)                         // 先頭bitが1であれば
    s16_yMag = u2_temp | 0b1110000000000000; // 上位を1で埋める 16bitの2の補数による負数
  else
    s16_yMag = u2_temp;

  u2_temp = 0;
  s16_zMag = 0;
  u2_temp = ((data[5] << 8) + data[4]) >> 1; //15bit
  if (u2_temp >> 14)                         // 先頭bitが1であれば
    s16_zMag = u2_temp | 0b1000000000000000; // 上位を1で埋める 16bitの2の補数による負数
  else
    s16_zMag = u2_temp;

  xMag = s16_xMag + s16_xMag_comp; //オフセット補正
  yMag = s16_yMag + s16_yMag_comp; //オフセット補正
  zMag = s16_zMag + s16_zMag_comp; //オフセット補正
}

void setup()
{
  pinMode(SDA_pin, INPUT_PULLUP); //デフォルトのPIN21,22を使用しない場合
  pinMode(SCL_pin, INPUT_PULLUP);
  Wire.begin(SDA_pin, SCL_pin);
  Serial.begin(9600);
  BMX055_Init();
  delay(300);
}

void loop()
{
  //BMX055 加速度の読み取り
  readAcc();
  Serial.print("Acc");
  Serial.print(",");
  Serial.print(f_xAcc);
  Serial.print(",");
  Serial.print(f_yAcc);
  Serial.print(",");
  Serial.println(f_zAcc);

  // //BMX055 ジャイロの読み取り
  readYaw();
  Serial.print("Yaw");
  Serial.print(",");
  Serial.print(f_xYaw);
  Serial.print(",");
  Serial.print(f_yYaw);
  Serial.print(",");
  Serial.println(f_zYaw);

  // //BMX055 磁気の読み取り
  readMag();
  Serial.print("Mag");
  Serial.print(",");
  Serial.print(xMag);
  Serial.print(",");
  Serial.print(yMag);
  Serial.print(",");
  Serial.println(zMag);

  delay(SAMPLING_TIME);
}
