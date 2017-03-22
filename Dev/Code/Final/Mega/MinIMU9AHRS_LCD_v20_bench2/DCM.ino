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

/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  float errord=0;
  float temporaryd[3][3];
  float renormd=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19
  
  errord= -Vector_Dot_Product(&DCM_Matrixd[0][0],&DCM_Matrixd[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Scale(&temporaryd[0][0], &DCM_Matrixd[1][0], errord); //eq.19
  Vector_Scale(&temporaryd[1][0], &DCM_Matrixd[0][0], errord); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Add(&temporaryd[0][0], &temporaryd[0][0], &DCM_Matrixd[0][0]);//eq.19
  Vector_Add(&temporaryd[1][0], &temporaryd[1][0], &DCM_Matrixd[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  Vector_Cross_Product(&temporaryd[2][0],&temporaryd[0][0],&temporaryd[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renormd= .5 *(3 - Vector_Dot_Product(&temporaryd[0][0],&temporaryd[0][0])); //eq.21
  Vector_Scale(&DCM_Matrixd[0][0], &temporaryd[0][0], renormd);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renormd= .5 *(3 - Vector_Dot_Product(&temporaryd[1][0],&temporaryd[1][0])); //eq.21
  Vector_Scale(&DCM_Matrixd[1][0], &temporaryd[1][0], renormd);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
  
  renormd= .5 *(3 - Vector_Dot_Product(&temporaryd[2][0],&temporaryd[2][0])); //eq.21
  Vector_Scale(&DCM_Matrixd[2][0], &temporaryd[2][0], renormd);
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  
  float mag_heading_xd;
  float mag_heading_yd;
  float errorCoursed;
  
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  
  static float Scaled_Omega_Pd[3];
  static float Scaled_Omega_Id[3];
  float Accel_magnituded;
  float Accel_weightd;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  
  Accel_magnituded = sqrt(Accel_Vectord[0]*Accel_Vectord[0] + Accel_Vectord[1]*Accel_Vectord[1] + Accel_Vectord[2]*Accel_Vectord[2]);
  Accel_magnituded = Accel_magnituded / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
  
  Accel_weightd = constrain(1 - 2*abs(1 - Accel_magnituded),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Cross_Product(&errorRollPitchd[0],&Accel_Vectord[0],&DCM_Matrixd[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_Pd[0],&errorRollPitchd[0],Kp_ROLLPITCH*Accel_weightd);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);

  Vector_Scale(&Scaled_Omega_Id[0],&errorRollPitchd[0],Ki_ROLLPITCH*Accel_weightd);
  Vector_Add(Omega_Id,Omega_Id,Scaled_Omega_Id);   
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  mag_heading_xd = cos(MAG_Headingd);
  mag_heading_yd = sin(MAG_Headingd);
  errorCoursed=(DCM_Matrixd[0][0]*mag_heading_yd) - (DCM_Matrixd[1][0]*mag_heading_xd);  //Calculating YAW error
  Vector_Scale(errorYawd,&DCM_Matrixd[2][0],errorCoursed); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_Pd[0],&errorYawd[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_Pd,Omega_Pd,Scaled_Omega_Pd);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  
  Vector_Scale(&Scaled_Omega_Id[0],&errorYawd[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_Id,Omega_Id,Scaled_Omega_Id);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
*/
/**************************************************/

void Matrix_update(void)
{
  Gyro_Vector[0]=Gyro_Scaled_X(gyro_x); //gyro x roll
  Gyro_Vector[1]=Gyro_Scaled_Y(gyro_y); //gyro y pitch
  Gyro_Vector[2]=Gyro_Scaled_Z(gyro_z); //gyro Z yaw
  
  Gyro_Vectord[0]=Gyro_Scaled_Xd(gyro_xd); //gyro x roll
  Gyro_Vectord[1]=Gyro_Scaled_Yd(gyro_yd); //gyro y pitch
  Gyro_Vectord[2]=Gyro_Scaled_Zd(gyro_zd); //gyro Z yaw
  
  Accel_Vector[0]=accel_x;
  Accel_Vector[1]=accel_y;
  Accel_Vector[2]=accel_z;
  
  Accel_Vectord[0]=accel_xd;
  Accel_Vectord[1]=accel_yd;
  Accel_Vectord[2]=accel_zd;
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
  
  Vector_Add(&Omegad[0], &Gyro_Vectord[0], &Omega_Id[0]);  //adding proportional term
  Vector_Add(&Omega_Vectord[0], &Omegad[0], &Omega_Pd[0]); //adding Integrator term
/////////////////////////////////
  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
  
 #if OUTPUTMODE==1         
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif
 
 #if OUTPUTMODE==1         
  Update_Matrixd[0][0]=0;
  Update_Matrixd[0][1]=-G_Dt*Omega_Vectord[2];//-z
  Update_Matrixd[0][2]=G_Dt*Omega_Vectord[1];//y
  Update_Matrixd[1][0]=G_Dt*Omega_Vectord[2];//z
  Update_Matrixd[1][1]=0;
  Update_Matrixd[1][2]=-G_Dt*Omega_Vectord[0];//-x
  Update_Matrixd[2][0]=-G_Dt*Omega_Vectord[1];//-y
  Update_Matrixd[2][1]=G_Dt*Omega_Vectord[0];//x
  Update_Matrixd[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrixd[0][0]=0;
  Update_Matrixd[0][1]=-G_Dt*Gyro_Vectord[2];//-z
  Update_Matrixd[0][2]=G_Dt*Gyro_Vectord[1];//y
  Update_Matrixd[1][0]=G_Dt*Gyro_Vectord[2];//z
  Update_Matrixd[1][1]=0;
  Update_Matrixd[1][2]=-G_Dt*Gyro_Vectord[0];
  Update_Matrixd[2][0]=-G_Dt*Gyro_Vectord[1];
  Update_Matrixd[2][1]=G_Dt*Gyro_Vectord[0];
  Update_Matrixd[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
  
  Matrix_Multiply(DCM_Matrixd,Update_Matrixd,Temporary_Matrixd); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
      
      DCM_Matrixd[x][y]+=Temporary_Matrixd[x][y];
    } 
  }
}

void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
  
  pitchd = -asin(DCM_Matrixd[2][0]);
  rolld = atan2(DCM_Matrixd[2][1],DCM_Matrixd[2][2]);
  yawd = atan2(DCM_Matrixd[1][0],DCM_Matrixd[0][0]);
}

