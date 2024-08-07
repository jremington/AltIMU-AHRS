# AltIMU-AHRS

Arduino Madgwick/Mahony AHRS fusion filters for the Pololu AltIMU9 and 10 series of 9 and 10DOF sensors. S. J. Remington 4/2020

UPDATE October 2023: Python code for calibrating magnetometer and accelerometer added, which optionally replaces Magneto. The results are identical with the two methods, however the Python program also produces a publication-quality plot of the corrected sensor data, which is handy for validating the results. Data input is a .csv file containing raw x,y,z magnetometer or accelerometer data, as for Magneto. The data file name is built into the code and can be changed. 

 references :
    -  https://teslabs.com/articles/magnetometer-calibration/      
    -  https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py

Clarify 12/17/2020: ** The standard sensor orientation is X North Y West, Z Up ** for conventional Tait-Bryan angles.

The current implementation has been tested only on the AltIMU10 v3 and will need minor modifications to take input from
other AltIMU sensor versions. The code supports the L3DG20 gyro and the LSM303 accelerometer/magnetometer.

The currently implemented AHRS algorithm is the standard Mahony scheme, which makes the most sense to me (although the 
Madgwick solver is included), as found and documented in several other IMU AHRS repositories. 

However, new features have been added, such as code to simplify accurate calibration of the accelerometer, 
magnetometer and gyro. Works very well, and is fast (180 updates/second with the Mahony algorithm, on a 16 MHz Arduino Pro Mini).

SENSOR CALIBRATION

The magnetometer and accelerometer calibration approaches are described in this excellent blog article:

    http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html 
    
I also strongly recommend this blog post as a general guide to magnetometer/accelerometer calibration

   https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

This repository in includes modified code for the Magneto program first described in the sailboatinstruments web page. 
It is a fair sized C program that runs on a PC or a Mac (suggest to use Code::Blocks IDE on Windows). 
For convenience, I’ve collected all of the basic parts of magneto, described in the sailboatinstruments link, into one file 
that can be compiled and run on a desktop.

The magneto approach fits an ellipsoid to the data, avoiding statistical problems associated with the min/max approach, 
rotates the ellipsoid to align with the coordinate axes, scales the axial dimensions to a sphere, and rotates back. 
The result is a set of offsets and a nine element matrix correction that must be applied to the raw data.

The magneto program was modified to add measurement rejection criteria and to publish data initialization statements 
that can be incorporated directly into the AHRS code.

See the folder calibration_examples for various intermediate files and screenshots of program output (.png image files). 

Below, 3D plot of corrected magnetometer data (mag3_raw.csv from the example), produced by calibrate3.py

![Capture](https://github.com/user-attachments/assets/85ab9d15-bc12-4485-9b8b-9a939c559421)


An example of magnetometer calibration using these procedures is presented and documented in some detail in this 
forum post: https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315

