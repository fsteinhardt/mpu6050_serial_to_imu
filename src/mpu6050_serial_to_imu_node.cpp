#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  uint8_t last_received_message_number;
  bool received_message = false;

  tf::Quaternion orientation;
  tf::Quaternion zero_orientation;

  std::string partial_line = "";

  ros::init(argc, argv, "mpu6050_serial_to_imu_node");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
  private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
  private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
  private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);


  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
  ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

  ros::Rate r(1000); // 1000 hz

  sensor_msgs::Imu imu;

  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;

  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
        if(ser.available())
        {
          std_msgs::String result;
          result.data = ser.readline(ser.available(), "\n");

          std::string input = partial_line + result.data;

          if (input.at( input.length() - 1 ) == '\n')
          {
            // line complete, delete partial_line var
            partial_line = "";

            // TODO: check if line is long enough??


            // parse line
            if (input.compare(0,2,"$\x03") == 0 && (input.size() == 26))
            {
              // get quaternion values
              int16_t w = (((0xff &(char)input[2]) << 8) | 0xff &(char)input[3]);
              ROS_DEBUG("w = %04x", w );

              int16_t x = (((0xff &(char)input[4]) << 8) | 0xff &(char)input[5]);
              ROS_DEBUG("x = %04x", x );

              int16_t y = (((0xff &(char)input[6]) << 8) | 0xff &(char)input[7]);
              ROS_DEBUG("y = %04x", y );

              int16_t z = (((0xff &(char)input[8]) << 8) | 0xff &(char)input[9]);
              ROS_DEBUG("z = %04x", z );

              double wf = w/16384.0;
              double xf = x/16384.0;
              double yf = y/16384.0;
              double zf = z/16384.0;

              tf::Quaternion orientation(xf, yf, zf, wf);

              if (!zero_orientation_set)
              {
                zero_orientation = orientation;
                zero_orientation_set = true;
              }

              //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
              tf::Quaternion differential_rotation;
              differential_rotation = zero_orientation.inverse() * orientation;



              // get gyro values
              int16_t gx = (((0xff &(char)input[10]) << 8) | 0xff &(char)input[11]);
              ROS_DEBUG("gx = %04x", gx );

              int16_t gy = (((0xff &(char)input[12]) << 8) | 0xff &(char)input[13]);
              ROS_DEBUG("gy = %04x", gy );

              int16_t gz = (((0xff &(char)input[14]) << 8) | 0xff &(char)input[15]);
              ROS_DEBUG("gz = %04x", gz );

              // calculate rotational velocities in rad/s
              // without the last factor the velocities were too small
              // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
              // FIFO frequency 100 Hz -> factor 10
              //TODO: check / test if rotational velocities are correct
              double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 10;
              double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 10;
              double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 10;


              // get acelerometer values
              int16_t ax = (((0xff &(char)input[16]) << 8) | 0xff &(char)input[17]);
              ROS_DEBUG("ax = %04x", ax );

              int16_t ay = (((0xff &(char)input[18]) << 8) | 0xff &(char)input[19]);
              ROS_DEBUG("ay = %04x", ay );

              int16_t az = (((0xff &(char)input[20]) << 8) | 0xff &(char)input[21]);
              ROS_DEBUG("az = %04x", az );

              // calculate accelerations in m/s²
              double axf = ax * (8.0 / 65536.0) * 9.81;
              double ayf = ay * (8.0 / 65536.0) * 9.81;
              double azf = az * (8.0 / 65536.0) * 9.81;


              if (received_message)
              {
                int message_distance = ((uint8_t)input[23]) - last_received_message_number;
                if ( message_distance != 1 )
                {
                  ROS_INFO("message number: %i", last_received_message_number);
                  ROS_WARN_STREAM("Missed " << message_distance << " MPU6050 data packets from arduino.");
                }
              }
              else
              {
                received_message = true;
              }
              last_received_message_number = (uint8_t)input[23];

              // calculate measurement time
              ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

              // publish imu message
              imu.header.stamp = measurement_time;
              imu.header.frame_id = frame_id;

              quaternionTFToMsg(differential_rotation, imu.orientation);

              imu.angular_velocity.x = gxf;
              imu.angular_velocity.y = gyf;
              imu.angular_velocity.z = gzf;

              imu.linear_acceleration.x = axf;
              imu.linear_acceleration.y = ayf;
              imu.linear_acceleration.z = azf;

              imu_pub.publish(imu);

              // publish tf transform
              if (broadcast_tf)
              {
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setRotation(differential_rotation);
                br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
              }
            }
          }
          else
          {
            // line incomplete, remember already received characters
            partial_line = input;
          }
        }
        else  // ser not available
        {

        }
      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(115200);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized.");
        }
        else
        {
          //ROS_INFO_STREAM("Could not initialize serial port.");
        }

      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}

