
From <http://davidegironi.blogspot.com.es/2013/02/avr-atmega-mpu6050-gyroscope-and.html>  

==================================================
MPU6050 lib
==================================================
= Copyright (c) Davide Gironi, 2012              =
= http://davidegironi.blogspot.it/               =
==================================================


The MPU-60X0 has an embedded 3-axis MEMS gyroscope, a 3-axis MEMS accelerometer,
and a Digital Motion Processor (DMP) hardware accelerator engine with an
auxiliary I2C port that interfaces to 3rd party digital sensors such
as magnetometers.

Two attitude extimation mode are implemented:
 * internal DMP processor
 * mahony filter

Gyro calibration is suggested to get more accurate parameters. To calibrate the
device read the raw values using the get non calibrated mode then calculate
the offset.

Setup parameters are stored in file mpu6050.h


Devel Notes
-----------
This library was developed on Eclipse, built with avr-gcc on Atmega168 @ 16MHz.
If you include <math.h> add libm library to linker.


License
-------
Please refer to LICENSE file for licensing information.
