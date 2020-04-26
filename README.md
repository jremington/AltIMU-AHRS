# AltIMU-AHRS

Madgwick/Mahoney AHRS filters for the Pololu AltIMU9 and 10 series of 9 and 10DOF sensors. S. J. Remington 4/2020
The current implementation has been tested only on the AltIMU10 v3 and will need modifications to take input from
later sensor versions.

The currently implemented AHRS algorithm is the standard Madgwick/Mahony scheme (two alternative solvers) found in 
other IMU repositories. However, new features have been added, such as code to simplify accurate calibration of the accelerometer, 
magnetometer and gyro. Works very well!

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

An example of magnetometer calibration using these procedures is presented and documented in some detail in this 
forum post: https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315

