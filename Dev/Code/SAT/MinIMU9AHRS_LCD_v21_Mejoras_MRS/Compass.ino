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

void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  float MAG_Xd;
  float MAG_Yd;
  float cos_rolld;
  float sin_rolld;
  float cos_pitchd;
  float sin_pitchd;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  cos_rolld = cos(rolld);
  sin_rolld = sin(rolld);
  cos_pitchd = cos(pitchd);
  sin_pitchd = sin(pitchd);
  
  // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
  c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
  c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
  c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;
  
  c_magnetom_xd = (float)(magnetom_xd - SENSOR_SIGNd[6]*M_X_MINd) / (M_X_MAXd - M_X_MINd) - SENSOR_SIGNd[6]*0.5;
  c_magnetom_yd = (float)(magnetom_yd - SENSOR_SIGNd[7]*M_Y_MINd) / (M_Y_MAXd - M_Y_MINd) - SENSOR_SIGNd[7]*0.5;
  c_magnetom_zd = (float)(magnetom_zd - SENSOR_SIGNd[8]*M_Z_MINd) / (M_Z_MAXd - M_Z_MINd) - SENSOR_SIGNd[8]*0.5;
  
  // Tilt compensated Magnetic filed X:
  MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
  
  MAG_Xd = c_magnetom_xd*cos_pitchd+c_magnetom_yd*sin_rolld*sin_pitchd+c_magnetom_zd*cos_rolld*sin_pitchd;
  // Tilt compensated Magnetic filed Y:
  MAG_Yd = c_magnetom_yd*cos_rolld-c_magnetom_zd*sin_rolld;
  // Magnetic Heading
  MAG_Headingd = atan2(-MAG_Yd,MAG_Xd);
}
