// program to collect raw data from MPU9250 accel and magnetometer for later correction.
// output in form suitable for .csv file
// The gyro offsets are calculated by simple averaging.

// Use Magneto 1.2 to calculate corrections for BOTH accelerometer and magnetometer
// http://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html
// S. J. Remington 3/2020

//USAGE:
// 1. Allow the sensor to sit still upon program startup for gyro bias data collection.
// 2. TURN THE SENSOR VERY SLOWLY AND CAREFULLY DURING DATA COLLECTION TO AVOID EXCESS ACCELERATION!


//The L3GD20H default full scale setting is +/- 245 dps.
//for a conversion factor of 8.75 mdps/LSB (least significant bit)

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

L3G gyro;
LSM303 compass;

#define sample_num  300 //acc/mag scaling points to collect

//raw data as vector
float Axyz[3] = {0};
float Gxyz[3] = {0};
float Mxyz[3] = {0};

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("AltIMU-10 V3 calibration data collection");

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


  int N = sample_num;


  Serial.println("Gyro bias collection ... KEEP SENSOR STILL");
  delay(2000);
  for (int i = 0; i < N; i++) {
    gyro.read();
    Gxyz[0] += float(gyro.g.x);
    Gxyz[1] += float(gyro.g.y);
    Gxyz[2] += float(gyro.g.z);
  }
  Serial.print("Done. RAW Gyro offsets ");
  Serial.print(Gxyz[0] / N, 1);
  Serial.print(", ");
  Serial.print(Gxyz[1] / N, 1);
  Serial.print(", ");
  Serial.print(Gxyz[2] / N, 1);
  Serial.println();

  Serial.print("Collecting ");
  Serial.print(N);
  Serial.println(" points for scaling, 3/second");
  Serial.println("TURN SENSOR VERY SLOWLY AND CAREFULLY IN 3D");
  delay(2000);

  float M_mag = 0, A_mag = 0;
  int i, j;
  j = N;
  while (j-- >= 0) {
    compass.read();
    Axyz[0] = compass.a.x;
    Axyz[1] = compass.a.y;
    Axyz[2] = compass.a.z;
    Mxyz[0] = compass.m.x;
    Mxyz[1] = compass.m.y;
    Mxyz[2] = compass.m.z;
    for (i = 0; i < 3; i++) {
      M_mag += Mxyz[i] * Mxyz[i];
      A_mag += Axyz[i] * Axyz[i];
    }
    Serial.print(compass.a.x);
    Serial.print(",");
    Serial.print(compass.a.y);
    Serial.print(",");
    Serial.print(compass.a.z);
    Serial.print(",");
    Serial.print(compass.m.x);
    Serial.print(",");
    Serial.print(compass.m.y);
    Serial.print(",");
    Serial.println(compass.m.z);
    delay(300);
  }
  Serial.print("Done. rms Acc = ");
  Serial.print(sqrt(A_mag / N));
  Serial.print(", rms Mag = ");
  Serial.println(sqrt(M_mag / N));
}


void loop() {}
