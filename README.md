mpu6050_serial_to_imu
=

This is a simple ROS node to connect an InvenSense MPU 6050 IMU  (I used a cheap GY-521 breakout board) to ROS.

It needs the MPU6050 example script MPU6050_DMP6.ino (part of the [i2cdevlib](http://www.i2cdevlib.com/) from Jeff Rowberg ) running on an arduino. The ROS node reads the IMU orientation from the serial port and publishes it as a ROS [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) message and a [tf transform](http://wiki.ros.org/tf).

This arduino script uses the MPU6050 DMP (digital motion processor?) to get filtered orientation values.

The ROS node only sets the orientation, no accelerations, velocities or covariance matrices are set.

Setup
=

To get the arduino demoscripot MPU6050_DMP6.ino programmed on your Arduino and for the electrical connections to the IMU see http://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/

You have to change the output in MPU6050_DMP6.ino to TEAPOT (uncomment "OUTPUT_TEAPOT" in MPU6050_DMP6.ino).

You might want to measure and set the Offsets for your MPU6050 chip: http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/

A demo launchfile is included to start the node and display the result in rviz.




#### Published Topics

* **`imu`** ([sensor_msgs::Imu])

	The resulting Imu orientation.

#### Published TF Transforms

*	The resulting orientation is published as a tf transform, the frame names can be set using the parameters.


#### Services

* **`set_zero_orientation`** ([std_srvs/Empty])

	This service sets the current orientation as the new zero orientation so that from now on only the difference to this orientation is sent.


#### Parameters

* **`port`** (string, default: "/dev/ttyACM0")

	The name of the serial port.

* **`time_offset_in_seconds`** (double, default: 0.0)

	This sets an offset which is added to the header timestamp of the imu-topic and the TF transforms. This can be used to synchronise the IMUs orientation values to values of another sensor.


* **`imu_frame_id`** (string, default: "imu_base")

	Sets the name of the base frame for imu messages.


* **`tf_parent_frame_id`** (string, default: "imu_base")

	Sets the name of the parent frame in the tf transform.


* **`tf_frame_id`** (string, default: "imu")

	Sets the name of the own frame in the tf transform.
