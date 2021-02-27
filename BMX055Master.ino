
//参考
//http://akizukidenshi.com/catalog/g/gK-13010/
//https://analogicintelligence.blogspot.com/2020/01/ae-bmx055_21.html

//ESP-WROOM-02はintが4バイトのためshortに修正

#include<Wire.h>

//JP1,JP2,JP3 = Open
#define Addr_Accl 0x19 //加速度センサのI2Cアドレス
#define Addr_Gyro 0x69 //ジャイロセンサのI2Cアドレス
#define Addr_Mag 0x13 //磁気センサのI2Cアドレス

int SDA_pin = 26;
int SCL_pin = 25;

float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;

//補正値
float xAccl_comp = 0.00;
float yAccl_comp = 0.00;
float zAccl_comp = 0.00;

float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;

//補正値
float xGyro_comp = 0.00;
float yGyro_comp = 0.00;
float zGyro_comp = 0.00;

short int xMag = 0;
short int yMag = 0;
short int zMag = 0;

//補正値
short int xMag_comp = 0;
short int yMag_comp = 0;
short int zMag_comp = 0;

void setup()
{
  pinMode(SDA_pin, INPUT_PULLUP); //デフォルトのPIN21,22を使用しない場合
  pinMode(SCL_pin, INPUT_PULLUP);

  // Wire(Arduino-I2C)の初期化
  Wire.begin(SDA_pin, SCL_pin); //デフォルトのPIN21,22を使用しない場合
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(300);
}

void loop()
{
  //Serial.println("--------------------------------------");

  //BMX055 加速度の読み取り
  BMX055_Accl();
  Serial.print("BMX055_Accl");
  Serial.print(",");
  Serial.print(xAccl);
  Serial.print(",");
  Serial.print(yAccl);
  Serial.print(",");
  Serial.println(zAccl);


  //BMX055 ジャイロの読み取り
  BMX055_Gyro();
  Serial.print("BMX055_Gyro");
  Serial.print(",");
  Serial.print(xGyro);
  Serial.print(",");
  Serial.print(yGyro);
  Serial.print(",");
  Serial.println(zGyro);

  //BMX055 磁気の読み取り

  BMX055_Mag();
  Serial.print("BMX055_Mag");
  Serial.print(",");
  Serial.print(xMag);
  Serial.print(",");
  Serial.print(yMag);
  Serial.print(",");
  Serial.println(zMag);

  Serial.println("");

  delay(1000);
}

//=====================================================================================//
void BMX055_Init()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);

  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();

}
//=====================================================================================//
void BMX055_Accl()
{
  short int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((0x02 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] << 8 ) + (data[0] & 0xF0)) >> 4;
  //xAccl msb(8bit)<<8 + xAccl lsb(4bit+****)
  //(8bit+4bit+****) >> 4
  //(0000+8bit+4bit)
  //  Serial.print(data[1] << 8, BIN);
  //  Serial.print(',');
  //  Serial.print(data[0] & 0xF0, BIN);
  //  Serial.print(',');
  //  Serial.print((short int)xAccl, BIN);
  //  Serial.print(',');
  //  Serial.println((short int)xAccl-pow(2, 12));
  if (xAccl >= pow(2, 11))  xAccl -= pow(2, 12);
  yAccl = ((data[3] << 8) + (data[2] & 0xF0)) >> 4;
  if (yAccl >= pow(2, 11))  yAccl -= pow(2, 12);
  zAccl = ((data[5] << 8) + (data[4] & 0xF0)) >> 4;
  if (zAccl >= pow(2, 11))  zAccl -= pow(2, 12);

  xAccl = xAccl * 0.00098; // renge +-2g
  yAccl = yAccl * 0.00098; // renge +-2g
  zAccl = zAccl * 0.00098; // renge +-2g

  xAccl += xAccl_comp;//オフセット補正
  yAccl += yAccl_comp;//オフセット補正
  zAccl += zAccl_comp;//オフセット補正
}
//=====================================================================================//
void BMX055_Gyro()
{
  short int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((0x02 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] << 8) + data[0];
  if (xGyro >= pow(2, 15))  xGyro -= pow(2, 16);
  yGyro = (data[3] << 8) + data[2];
  if (yGyro >= pow(2, 15))  yGyro -= pow(2, 16);
  zGyro = (data[5] << 8) + data[4];
  if (zGyro >= pow(2, 15))  zGyro -= pow(2, 16);

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s

  xGyro += xGyro_comp;//オフセット補正
  yGyro += yGyro_comp;//オフセット補正
  zGyro += zGyro_comp;//オフセット補正
}

//=====================================================================================//
void BMX055_Mag()
{
  short int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb, (Hall lsb, Hall msb)
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }

  // Convert the data
  xMag = ((data[1] << 8) | data[0]) >> 3;
  if (xMag >= pow(2, 12))  xMag -= pow(2, 13);
  yMag = ((data[3] << 8) | data[2]) >> 3;
  if (yMag >= pow(2, 12))  yMag -= pow(2, 13);
  zMag = ((data[5] << 8) | data[4]) >> 1;
  if (zMag >= pow(2, 14))  zMag -= pow(2, 15);

  xMag += xMag_comp;//オフセット補正
  yMag += yMag_comp;//オフセット補正
  zMag += zMag_comp;//オフセット補正
}
