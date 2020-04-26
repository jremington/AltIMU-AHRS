//
// Tilt compensated compass  S.J. Remington 3/2020
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
//
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for magneto, use the companion program MPU9250_cal
//
// Below I use the diagonal element of matrix A and ignore the off diagonal components.
// If those off diagonal terms are large, (most likely only for the magnetometer)
// add those terms in to the correction routines getMag_cal() and getAcc_cal()
//
// This version must be compiled with library routines in subfolder "libs"
#include <Wire.h>
#include <LSM303.h>

LSM303 compass;

// VERY VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using
// altimu10v3_cal and Magneto
//The scale constants should *approximately* normalize the vector magnitude to 1.0
//The compass will NOT work well or at all if these are not correct
float M_B[3] {   53.49,  -43.59,   43.77}; //mag offsets and scale
float M_Ainv[3][3]
{ {  0.94740,  0.01035,  0.01707},
  {  0.01035,  0.94300, -0.00171},
  {  0.01707, -0.00171,  1.14336}
};

float A_B[3] { -585.81,  122.99, -107.34}; //accelerometer
float A_Ainv[3][3]
{ {  0.62071,  0.00324,  0.00382},
  {  0.00324,  0.60508, -0.00098},
  {  0.00382, -0.00098,  0.59250}
};

//raw data and scaled as vector
float Axyz[3];
float Mxyz[3];


/*
  This tilt-compensated code assumes that the sensor is oriented with Mag X pointing
  to the North, Y pointing East, and Z pointing down (toward the ground).
  With the MPU9250, the accelerometer is aligned differently, so the accelerometer axes are swapped.

  The code compensates for tilts of up to 90 degrees away from horizontal.
  Facing vector p is the direction of travel and allows reassigning these directions.
  It should be defined as pointing forward,
  parallel to the ground, with coordinates {X, Y, Z} (in magnetometer frame of reference).
*/
float p[] = {1, 0, 0}; //X ahead, to North

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial); //wait for connection if necessary
  // initialize device
  if (!compass.init()) {
    Serial.println("compass init failure");
    while (1);
  }
  Serial.println("AltIMU-10 v3 tilt compensated compass");
}

void loop()
{
  get_IMU_scaled(); //latest accelerometer reading, offset and scale corrected

  Serial.println(get_heading(Axyz, Mxyz, p));

  delay(1000);
}
void vector_cross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}


// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(float acc[3], float mag[3], float p[3])
{
  float E[3], N[3]; //direction vectors

  // cross "down" (acceleration vector) with magnetic vector (magnetic north + inclination) with  to produce "east"
  vector_cross(acc, mag, E);
  vector_normalize(E);

  // cross "east" with "down" to produce "north" (parallel to the ground)
  vector_cross(E, acc, N);
  vector_normalize(N);

  // compute heading
  float heading = atan2(vector_dot(E, p), vector_dot(N, p)) * 180 / M_PI;
  heading = -heading;   //conventional nav, yaw increases CW from North to East
  // http://www.ngdc.noaa.gov/geomag-web/#declination
  heading += 14.9; //correction for local magnetic declination
  if (heading > 360.0) heading -= 360.0;
  if (heading < 0.0)  heading += 360.0;
  return (int) (heading + 0.5);  //round up
}

void get_IMU_scaled(void) {
  byte i;
  float temp[3];
  compass.read();
  Axyz[0] = compass.a.x;
  Axyz[1] = compass.a.y;
  Axyz[2] = compass.a.z;
  Mxyz[0] = compass.m.x;
  Mxyz[1] = compass.m.y;
  Mxyz[2] = compass.m.z;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply offsets (bias) and scale factors from Magneto
  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
  /*
    // debug prints
    Serial.print(Axyz[0]);
    Serial.print(", ");
    Serial.print(Axyz[1]);
    Serial.print(", ");
    Serial.print(Axyz[2]);
    Serial.print(", ");
    Serial.print(Mxyz[0]);
    Serial.print(", ");
    Serial.print(Mxyz[1]);
    Serial.print(", ");
    Serial.print(Mxyz[2]);
    Serial.println();
  */
}
