#include <Wire.h>
 
#define BNO055_ADDR             (0x28)
#define BNO055_CHIP_ID          0x00
#define BNO055_CHIP_ID_VALUE    0xa0
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_OPR_MODE         0x3d
#define CONFIGMODE              0x00
#define MODE_NDOF               0x0c
#define ACCEL_OFFSET_X_LSB      0x55
#define ACCEL_OFFSET_X_MSB      0x56
#define ACCEL_OFFSET_Y_LSB      0x57
#define ACCEL_OFFSET_Y_MSB      0x58
#define ACCEL_OFFSET_Z_LSB      0x59
#define ACCEL_OFFSET_Z_MSB      0x5a
#define MAG_OFFSET_X_LSB        0x5b
#define MAG_OFFSET_X_MSB        0x5c
#define MAG_OFFSET_Y_LSB        0x5d
#define MAG_OFFSET_Y_MSB        0x5e
#define MAG_OFFSET_Z_LSB        0x5f
#define MAG_OFFSET_Z_MSB        0x60
#define GYRO_OFFSET_X_LSB       0x61
#define GYRO_OFFSET_X_MSB       0x62
#define GYRO_OFFSET_Y_LSB       0x63
#define GYRO_OFFSET_Y_MSB       0x64
#define GYRO_OFFSET_Z_LSB       0x65
#define GYRO_OFFSET_Z_MSB       0x66
#define ACCEL_RADIUS_LSB        0x67
#define ACCEL_RADIUS_MSB        0x68
#define MAG_RADIUS_LSB          0x69
#define MAG_RADIUS_MSB          0x6a
#define BNO055_EULER_H_LSB      0x1a
#define BNO055_EULER_H_MSB      0x1b
#define BNO055_EULER_R_LSB      0x1c
#define BNO055_EULER_R_MSB      0x1d
#define BNO055_EULER_P_LSB      0x1e
#define BNO055_EULER_P_MSB      0x1f
 
void I2C_BNO055_INIT()
{
  char data[7];
  char chip_id;
   
  char accel_offset_x_lsb_value = 8;
  char accel_offset_x_msb_value = 0;
  char accel_offset_y_lsb_value = 34;
  char accel_offset_y_msb_value = 0;
  char accel_offset_z_lsb_value = 7;
  char accel_offset_z_msb_value = 0;
 
  char mag_offset_x_lsb_value = 45;
  char mag_offset_x_msb_value = 255;
  char mag_offset_y_lsb_value = 116;
  char mag_offset_y_msb_value = 0;
  char mag_offset_z_lsb_value = 90;
  char mag_offset_z_msb_value = 1;
 
  char gyro_offset_x_lsb_value = 1;
  char gyro_offset_x_msb_value = 0;
  char gyro_offset_y_lsb_value = 1;
  char gyro_offset_y_msb_value = 0;
  char gyro_offset_z_lsb_value = 0;
  char gyro_offset_z_msb_value = 0;
 
  char accel_radius_lsb_value = 0;
  char accel_radius_msb_value = 3;
  char mag_radius_lsb_value = 66;
  char mag_radius_msb_value = 2;
   
  data[0] = BNO055_CHIP_ID;
 
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(data, 1);
  Wire.endTransmission(false);
 
  Wire.requestFrom(BNO055_ADDR, 7, true);
  Serial.println(Wire.available());
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();
  data[4] = Wire.read();
  data[5] = Wire.read();
  data[6] = Wire.read();
 
  chip_id = data[0];
 
  while (chip_id != BNO055_CHIP_ID_VALUE)
  {
    data[0] = BNO055_CHIP_ID;
 
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(data, 1);
    Wire.endTransmission(false);
 
   
    Wire.requestFrom(BNO055_ADDR, 7, true);
       
    Serial.println(Wire.available());
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
    data[6] = Wire.read();
 
    chip_id = data[0];
    yield();
  }
 
  data[0] = BNO055_OPR_MODE;
  data[1] = CONFIGMODE;
 
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(data,2);
  Wire.endTransmission(true);
 
  delay(50);
 
  data[0] = BNO055_AXIS_MAP_CONFIG;
  data[1] = 0x24;
  data[2] = 0x00;
 
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(data,2);
  Wire.endTransmission(true);
   
  data[0] = ACCEL_OFFSET_X_LSB; data[1] = accel_offset_x_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = ACCEL_OFFSET_X_MSB; data[1] = accel_offset_x_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = ACCEL_OFFSET_Y_LSB; data[1] = accel_offset_y_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = ACCEL_OFFSET_Y_LSB; data[1] = accel_offset_y_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = ACCEL_OFFSET_Z_LSB; data[1] = accel_offset_z_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = ACCEL_OFFSET_Z_LSB; data[1] = accel_offset_z_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
   
  data[0] = MAG_OFFSET_X_LSB; data[1] = mag_offset_x_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = MAG_OFFSET_X_MSB; data[1] = mag_offset_x_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = MAG_OFFSET_Y_LSB; data[1] = mag_offset_y_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = MAG_OFFSET_Y_LSB; data[1] = mag_offset_y_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = MAG_OFFSET_Z_LSB; data[1] = mag_offset_z_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = MAG_OFFSET_Z_LSB; data[1] = mag_offset_z_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
   
  data[0] = GYRO_OFFSET_X_LSB; data[1] = gyro_offset_x_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = GYRO_OFFSET_X_MSB; data[1] = gyro_offset_x_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = GYRO_OFFSET_Y_LSB; data[1] = gyro_offset_y_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = GYRO_OFFSET_Y_LSB; data[1] = gyro_offset_y_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = GYRO_OFFSET_Z_LSB; data[1] = gyro_offset_z_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = GYRO_OFFSET_Z_LSB; data[1] = gyro_offset_z_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
   
  data[0] = ACCEL_RADIUS_LSB; data[1] = accel_radius_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = ACCEL_RADIUS_MSB; data[1] = accel_radius_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = MAG_RADIUS_LSB; data[1] = mag_radius_lsb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
  data[0] = MAG_RADIUS_MSB; data[1] = mag_radius_msb_value; Wire.beginTransmission(BNO055_ADDR); Wire.write(data,2); Wire.endTransmission(true); delay(20);
   
  data[0] = BNO055_OPR_MODE;
  data[1] = MODE_NDOF;
 
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(data,2);
  Wire.endTransmission(true);
 
  delay(10);
}
 
int I2C_BNO055_READ_YAW()
{
  int16_t wert;
  char data[6];
 
  data[0] = BNO055_EULER_H_LSB;
 
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(data, 1);
  Wire.endTransmission(false);
 
  Wire.requestFrom(BNO055_ADDR, 2, true);
  data[0] = ((Wire.available() > 0) ? Wire.read() : 0);
  data[1] = ((Wire.available() > 0) ? Wire.read() : 0);
  data[2] = ((Wire.available() > 0) ? Wire.read() : 0);
  data[3] = ((Wire.available() > 0) ? Wire.read() : 0);
  data[4] = ((Wire.available() > 0) ? Wire.read() : 0);
  data[5] = ((Wire.available() > 0) ? Wire.read() : 0);
 
  wert = data[1] << 8 | data[0];
  wert = (double)wert/16;
  return wert;
}
 
void setup() {
  Serial.begin(9600); /* begin serial for debug */
  Wire.begin(D1, D2); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
  I2C_BNO055_INIT();
}
 
void loop() {
  Serial.print("YAW:");
  Serial.println(I2C_BNO055_READ_YAW());
 
  delay(200);
}