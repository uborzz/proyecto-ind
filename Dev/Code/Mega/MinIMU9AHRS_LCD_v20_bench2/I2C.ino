/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

#include <L3G.h>
#include <LSM303.h>

L3G gyro, gyroD;
LSM303 compass, compassD;

LSM303::deviceType devAuto=(LSM303::deviceType) 4;//Auto
LSM303::sa0State saHigh=(LSM303::sa0State) 1; //High
LSM303::sa0State saLow=(LSM303::sa0State) 0; //Low

L3G::deviceType devAutoL3G=(L3G::deviceType) 3;//Auto
L3G::sa0State saHighL3G=(L3G::sa0State) 1; //High
L3G::sa0State saLowL3G=(L3G::sa0State) 0; //Low

void I2C_Init()
{
  Wire.begin();
}

void Gyro_Init()
{

  gyro.init(devAutoL3G,saHighL3G);
  gyroD.init(devAutoL3G,saLowL3G);
  gyro.enableDefault();
  gyroD.enableDefault();
  gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyroD.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
  gyroD.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void Read_Gyro()
{
  gyro.read();
  
  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
  
  gyroD.read();
  
  ANd[0] = gyroD.g.x;
  ANd[1] = gyroD.g.y;
  ANd[2] = gyroD.g.z;
  gyro_xd = SENSOR_SIGNd[0] * (ANd[0] - AN_OFFSETd[0]);
  gyro_yd = SENSOR_SIGNd[1] * (ANd[1] - AN_OFFSETd[1]);
  gyro_zd = SENSOR_SIGNd[2] * (ANd[2] - AN_OFFSETd[2]);
}

void Accel_Init()
{
  //compass.initH();
  ///////////PRUEBA una sola funcion init/////////////////
  compass.init(devAuto,saHigh);//devauto,pinSa0 high
  compassD.init(devAuto,saLow);
  
  compass.enableDefault();
  compassD.enableDefault();
  switch (compass.getDeviceType())
  {
    case LSM303::device_D:
      compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
  
  switch(compassD.getDeviceType())
  {
    case LSM303::device_D:
      compassD.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      compassD.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      compassD.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
  compass.readAcc();
  
  AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = compass.a.y >> 4;
  AN[5] = compass.a.z >> 4;
  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
  
  compassD.readAcc();
  
  ANd[3] = compassD.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  ANd[4] = compassD.a.y >> 4;
  ANd[5] = compassD.a.z >> 4;
  accel_xd = SENSOR_SIGNd[3] * (ANd[3] - AN_OFFSETd[3]);
  accel_yd = SENSOR_SIGNd[4] * (ANd[4] - AN_OFFSETd[4]);
  accel_zd = SENSOR_SIGNd[5] * (ANd[5] - AN_OFFSETd[5]);
}

void Compass_Init()
{
  // doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
}

void Read_Compass()
{
  compass.readMag();
  
  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
  
  compassD.readMag();
  
  magnetom_xd = SENSOR_SIGNd[6] * compassD.m.x;
  magnetom_yd = SENSOR_SIGNd[7] * compassD.m.y;
  magnetom_zd = SENSOR_SIGNd[8] * compassD.m.z;
}

