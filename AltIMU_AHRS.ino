//
// AltIMU-10 v3 Magwick/Mahony AHRS  S.J. Remington 
// last update 12/17/2020, clean up comments
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Standard sensor orientation: Z Up X North Y West

// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work, and the gyro offset must be determned.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
// To collect data for calibration, use the companion programs altimu10v3_cal and Magneto 1.2 from sailboatinstruments.blogspot.com
//

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

L3G gyro;
LSM303 compass;

// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//These are the previously determined offsets (bias) and scale factors for accelerometer and magnetometer,
// using altimu10v3_cal and Magneto 1.2 or 1.3
//The AHRS will NOT work well or at all if these are not correct

float M_B[3] {   53.49,  -43.59,   43.77}; //mag offsets and scale
float M_Ainv[3][3]
{ {  0.94740,  0.01035,  0.01707},
  {  0.01035,  0.94300, -0.00171},
  {  0.01707, -0.00171,  1.14336}
};

float A_B[3] { -585.81,  122.99, -107.34}; //accelerometer offsets and scale
float A_Ainv[3][3]
{ {  0.62071,  0.00324,  0.00382},
  {  0.00324,  0.60508, -0.00098},
  {  0.00382, -0.00098,  0.59250}
};

//The L3GD20H default full scale setting is +/- 245 dps.
#define gscale 8.75e-3*(PI/180.0)  //gyro default 8.75 mdps -> rad/s

float G_off[3] = {83.7, -293.8, -380.5}; //raw offsets, determined for gyro at rest
// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

char s[60]; //snprintf buffer

// globals for AHRS loop timing
unsigned long now = 0, last = 0; //micros() timers
float deltat = 0;  //loop time in seconds
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long print_ms = 500; //print orientation angles every "print_ms" milliseconds

//raw data and scaled as vector
float Axyz[3];
float Gxyz[3];
float Mxyz[3];


// NOW USING MAHONY FILTER

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define Kp 30.0
#define Ki 0.0

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("AltIMU-10 V3 AHRS");

  Wire.begin();
  if (!compass.init()) {
    Serial.println("compass init failure");
    while (1);
  }
  compass.enableDefault();
  if (!gyro.init())
  {
    Serial.println("gyro init failure");
    while (1);
  }
  gyro.enableDefault();
}

// AHRS loop

void loop()
{
  static int i = 0, count=0;
  float qw, qx, qy, qz;

  get_IMU_scaled();
  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;

  MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                         Mxyz[0], Mxyz[1], Mxyz[2], deltat);
// Define Tait-Bryan angles.
//  Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.

  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));

  // to degrees
  yaw   *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  // http://www.ngdc.noaa.gov/geomag-web/#declination
  //conventional nav, yaw increases CW from North, corrected for local magnetic declination
  yaw = -yaw + 14.9;
  if (yaw > 360.0) yaw -= 360.0;
  if (yaw < 0) yaw += 360.0;
  count++; //loop iterations
  now_ms = millis(); //time to print?
  if (now_ms - last_ms >= print_ms)
  {
    last_ms = now_ms;
    // print angles for serial plotter...
    //  Serial.print("ypr ");
    Serial.print(yaw, 0);
    Serial.print(", ");
    Serial.print(pitch, 0);
    Serial.print(", ");
    Serial.println(roll, 0);
//    Serial.print(", ");
//    Serial.println(count); //number of loop iterations in this print interval (about 90 with 16 MHz Pro Mini)
    count=0;
  }
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
  gyro.read();
  Gxyz[0] = gyro.g.x;
  Gxyz[1] = gyro.g.y;
  Gxyz[2] = gyro.g.z;

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

  Gxyz[0] = ((float) gyro.g.x - G_off[0]) * gscale; //d/s to radians/s
  Gxyz[1] = ((float) gyro.g.y - G_off[1]) * gscale;
  Gxyz[2] = ((float) gyro.g.z - G_off[2]) * gscale;
  }

// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
 // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
// short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;
  /*
    // already done in loop()

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;
  */
  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms to gyro rate measurement
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
qa = q1;
qb = q2;
qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);
  
  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
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
