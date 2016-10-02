mpu6050_serial_to_imu
=

This is a simple ROS node to connect an InvenSense MPU 6050 IMU  (I used a cheap GY-521 breakout board) to ROS.

It uses an arduino running a program using the [i2cdevlib](http://www.i2cdevlib.com/) from Jeff Rowberg to read the measurements from the MPU6050 sensor and send them to the computer. The ROS node reads the IMU data from the arduinos serial port and publishes the linear accelerations, rotational velocities and the orientation as a ROS [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) message. The covariances in this message can be set using parameters. The orientation can also be broadcast as a [tf transform](http://wiki.ros.org/tf).

The arduino script uses the MPU6050 DMP (digital motion processor?) to get filtered orientation values.


Setup
=

See http://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/ for the electrical connections and how to setup the i2cdevlib.

Then program the arduino with the arduino/MPU6050/MPU6050.ino file from this repository.

You might want to measure and set the offsets for your MPU6050 chip: http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/.

After you have measured the offset values you can change them in the MPU6050.ino file (after the line `// supply your own gyro offsets here, scaled for min sensitivity`).

The launchfile demo.launch is included to start the node and display some imu values in rviz, but you might need to change the serial port name to point to your arduino.


#### Published Topics

* **`imu/data`** ([sensor_msgs::Imu])

	The measured accelerometer, gyro and orientation values.

* **`imu/temperature`** ([sensor_msgs::Temperature])

	The measured temperature in degrees Celsius.

#### Published TF Transforms

*	The resulting orientation can be published as a tf transform, the frame names can be set using parameters.


#### Services

* **`set_zero_orientation`** ([std_srvs/Empty])

	This service sets the current orientation as the new zero orientation so that from now on only the difference to this orientation is sent.


#### Parameters

* **`port`** (string, default: "/dev/ttyACM0")

	The name of the serial port.

* **`time_offset_in_seconds`** (double, default: 0.0)

	This sets an offset which is added to the header timestamp of the imu-topic and the TF transforms. This can be used to synchronise the IMUs orientation values to values of another sensor.


* **`broadcast_tf`** (bool, default: true)

	If true: publish the orientation of the IMU as a tf transform. The frame names can be set with the **tf_frame_id** and **tf_parent_frame_id** parameters.


* **`imu_frame_id`** (string, default: "imu_link")

	Sets the name of the frame for imu messages.


* **`tf_parent_frame_id`** (string, default: "imu_base")

	Sets the name of the parent frame in the tf transform.


* **`tf_frame_id`** (string, default: "imu_link")

	Sets the name of the own frame in the tf transform.

* **`linear_acceleration_stddev`** (double, default: 0.0)

	Sets the linear_acceleration_covariance matrix diagonal elements.

* **`angular_velocity_stddev`** (double, default: 0.0)

	Sets the angular_velocity_covariance matrix diagonal elements.

* **`orientation_stddev`** (double, default: 0.0)

	Sets the orientation_covariance matrix diagonal elements.

