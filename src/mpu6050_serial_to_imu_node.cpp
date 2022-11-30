#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Mpu6050SerialToImuNode : public rclcpp::Node
{
public:
  Mpu6050SerialToImuNode() : Node("imu"),
  m_owned_ctx(new drivers::common::IoContext(2)),
  m_serial_driver{new drivers::serial_driver::SerialDriver(*m_owned_ctx)}
  {
    get_params();
    configure();
    run();
  }

  void configure()
  {
    // Create Publisher
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
      "data", rclcpp::QoS{50}
      );

    imu_temperature_pub = this->create_publisher<sensor_msgs::msg::Temperature>(
      "temperature", rclcpp::QoS{50}
      );

    try {
      m_serial_driver->init_port(m_device_name, *m_device_config);
      if (!m_serial_driver->port()->is_open()) {
        m_serial_driver->port()->open();
        // m_serial_driver->port()->async_receive(
        //   std::bind(
        //     &Mpu6050SerialToImuNode::receive_callback, this, std::placeholders::_1,
        //     std::placeholders::_2));
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Error creating serial port: %s - %s",
        m_device_name.c_str(), ex.what());
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service =
      create_service<std_srvs::srv::Empty>("set_zero_orientation", 
      std::bind(&Mpu6050SerialToImuNode::set_zero_orientation, this, std::placeholders::_1, std::placeholders::_2));
  }

  int run() {
    uint8_t last_received_message_number;
    bool received_message = false;
    size_t data_packet_start;

    tf2::Quaternion orientation;
    tf2::Quaternion zero_orientation;

    rclcpp::Rate loop_rate(200); // 200 hz

    sensor_msgs::msg::Imu imu;

    imu.linear_acceleration_covariance[0] = linear_acceleration_stddev * linear_acceleration_stddev;
    imu.linear_acceleration_covariance[4] = linear_acceleration_stddev * linear_acceleration_stddev;
    imu.linear_acceleration_covariance[8] = linear_acceleration_stddev * linear_acceleration_stddev;

    imu.angular_velocity_covariance[0] = angular_velocity_stddev * angular_velocity_stddev;
    imu.angular_velocity_covariance[4] = angular_velocity_stddev * angular_velocity_stddev;
    imu.angular_velocity_covariance[8] = angular_velocity_stddev * angular_velocity_stddev;

    imu.orientation_covariance[0] = orientation_stddev * orientation_stddev;
    imu.orientation_covariance[4] = orientation_stddev * orientation_stddev;
    imu.orientation_covariance[8] = orientation_stddev * orientation_stddev;

    sensor_msgs::msg::Temperature temperature_msg;
    temperature_msg.variance = 0;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(0,0,0));

    std::string input;
    std::string read;
    std::vector<uint8_t> buffer;
    static auto temp_buffer = std::vector<uint8_t>(2048, 0);
    
    
    while(rclcpp::ok())
    {
      try
      {
        if (m_serial_driver->port()->is_open())
        {
          // read string from serial device
          size_t package_length = m_serial_driver->port()
            ->receive(temp_buffer);
          

          if(package_length)
          {
            read = std::string(temp_buffer.begin(), temp_buffer.end());
            read = read.substr(0, package_length);
            RCLCPP_DEBUG(get_logger(), "read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
            input += read;
            while (input.length() >= 28) // while there might be a complete package in input
            {
              //parse for data packets
              data_packet_start = input.find("$\x03");
              if (data_packet_start != std::string::npos)
              {
                RCLCPP_DEBUG(get_logger(), "found possible start of data packet at position %d", data_packet_start);
                if ((input.length() >= data_packet_start + 28) && (input.compare(data_packet_start + 26, 2, "\r\n") == 0))  //check if positions 26,27 exist, then test values
                {
                  RCLCPP_DEBUG(get_logger(), "seems to be a real data package: long enough and found end characters");
                  // get quaternion values
                  int16_t w = (((0xff &(char)input[data_packet_start + 2]) << 8) | 0xff &(char)input[data_packet_start + 3]);
                  int16_t x = (((0xff &(char)input[data_packet_start + 4]) << 8) | 0xff &(char)input[data_packet_start + 5]);
                  int16_t y = (((0xff &(char)input[data_packet_start + 6]) << 8) | 0xff &(char)input[data_packet_start + 7]);
                  int16_t z = (((0xff &(char)input[data_packet_start + 8]) << 8) | 0xff &(char)input[data_packet_start + 9]);

                  double wf = w/16384.0;
                  double xf = x/16384.0;
                  double yf = y/16384.0;
                  double zf = z/16384.0;

                  tf2::Quaternion orientation(xf, yf, zf, wf);

                  if (!zero_orientation_set)
                  {
                    zero_orientation = orientation;
                    zero_orientation_set = true;
                  }

                  //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
                  tf2::Quaternion differential_rotation;
                  differential_rotation = zero_orientation.inverse() * orientation;

                  // get gyro values
                  int16_t gx = (((0xff &(char)input[data_packet_start + 10]) << 8) | 0xff &(char)input[data_packet_start + 11]);
                  int16_t gy = (((0xff &(char)input[data_packet_start + 12]) << 8) | 0xff &(char)input[data_packet_start + 13]);
                  int16_t gz = (((0xff &(char)input[data_packet_start + 14]) << 8) | 0xff &(char)input[data_packet_start + 15]);
                  // calculate rotational velocities in rad/s
                  // without the last factor the velocities were too small
                  // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                  // FIFO frequency 100 Hz -> factor 10 ?
                  // seems 25 is the right factor
                  //TODO: check / test if rotational velocities are correct
                  double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                  double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                  double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

                  // get acelerometer values
                  int16_t ax = (((0xff &(char)input[data_packet_start + 16]) << 8) | 0xff &(char)input[data_packet_start + 17]);
                  int16_t ay = (((0xff &(char)input[data_packet_start + 18]) << 8) | 0xff &(char)input[data_packet_start + 19]);
                  int16_t az = (((0xff &(char)input[data_packet_start + 20]) << 8) | 0xff &(char)input[data_packet_start + 21]);
                  // calculate accelerations in m/s²
                  double axf = ax * (8.0 / 65536.0) * 9.81;
                  double ayf = ay * (8.0 / 65536.0) * 9.81;
                  double azf = az * (8.0 / 65536.0) * 9.81;

                  // get temperature
                  int16_t temperature = (((0xff &(char)input[data_packet_start + 22]) << 8) | 0xff &(char)input[data_packet_start + 23]);
                  double temperature_in_C = (temperature / 340.0 ) + 36.53;
                  RCLCPP_DEBUG_STREAM(get_logger(), "Temperature [in C] " << temperature_in_C);

                  uint8_t received_message_number = input[data_packet_start + 25];
                  RCLCPP_DEBUG(get_logger(), "received message number: %i", received_message_number);

                  if (received_message) // can only check for continuous numbers if already received at least one packet
                  {
                    uint8_t message_distance = received_message_number - last_received_message_number;
                    if ( message_distance > 1 )
                    {
                      RCLCPP_WARN_STREAM(get_logger(), "Missed " << message_distance - 1 << " MPU6050 data packets from arduino.");
                    }
                  }
                  else
                  {
                    received_message = true;
                  }
                  last_received_message_number = received_message_number;

                  // calculate measurement time
                  rclcpp::Time measurement_time = now() + rclcpp::Duration(time_offset_in_seconds);

                  // publish imu message
                  imu.header.stamp = measurement_time;
                  imu.header.frame_id = frame_id;

                  tf2::convert(differential_rotation, imu.orientation);

                  imu.angular_velocity.x = gxf;
                  imu.angular_velocity.y = gyf;
                  imu.angular_velocity.z = gzf;

                  imu.linear_acceleration.x = axf;
                  imu.linear_acceleration.y = ayf;
                  imu.linear_acceleration.z = azf;

                  imu_pub->publish(imu);

                  // publish temperature message
                  temperature_msg.header.stamp = measurement_time;
                  temperature_msg.header.frame_id = frame_id;
                  temperature_msg.temperature = temperature_in_C;

                  imu_temperature_pub->publish(temperature_msg);

                  // publish tf transform
                  if (broadcast_tf)
                  {
                    transform.setRotation(differential_rotation);
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped.header.stamp = measurement_time;
                    transformStamped.header.frame_id = tf_parent_frame_id;
                    transformStamped.child_frame_id = tf_frame_id;
                    transformStamped.transform.translation.x = transform.getOrigin().getX();
                    transformStamped.transform.translation.y = transform.getOrigin().getY();
                    transformStamped.transform.translation.z = transform.getOrigin().getZ();

                    transformStamped.transform.rotation.x = transform.getRotation().x();
                    transformStamped.transform.rotation.y = transform.getRotation().y();
                    transformStamped.transform.rotation.z = transform.getRotation().z();
                    transformStamped.transform.rotation.w = transform.getRotation().w();

                    tf_broadcaster_->sendTransform(transformStamped);
                  }
                  input.erase(0, data_packet_start + 28); // delete everything up to and including the processed packet
                }
                else
                {
                  if (input.length() >= data_packet_start + 28)
                  {
                    input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                  }
                  else
                  {
                    // do not delete start character, maybe complete package has not arrived yet
                    input.erase(0, data_packet_start);
                  }
                }
              }
              else
              {
                // no start character found in input, so delete everything
                input.clear();
              }
            }
          }
        } //TODO: When Serialport is closed try to open it
      }

      catch (asio::system_error& e)
      {
        RCLCPP_ERROR(get_logger(), "Error reading from the serial port %s. Closing connection.", m_device_name.c_str());
        m_serial_driver->port()->close();
      }
      loop_rate.sleep();
    }
    return 0;
    
  }

  bool set_zero_orientation(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Zero Orientation Set.");
    zero_orientation_set = false;
    return true;
  }

private:
  void get_params()
  {
    try {
      tf_parent_frame_id = declare_parameter<std::string>("tf_parent_frame_id", "imu_base");
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The tf_parent_frame_id value provided was invalid");
      throw ex;
    }
    try {
      tf_frame_id = declare_parameter<std::string>("tf_frame_id", "imu_link");
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The tf_frame_id value provided was invalid");
      throw ex;
    }
    try {
      frame_id = declare_parameter<std::string>("frame_id", "imu_link");
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The frame_id value provided was invalid");
      throw ex;
    }
    try {
      time_offset_in_seconds = declare_parameter<double>("time_offset_in_seconds", 0.0);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The time_offset_in_seconds value provided was invalid");
      throw ex;
    }
    try {
      broadcast_tf = declare_parameter<bool>("broadcast_tf", true);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The broadcast_tf value provided was invalid");
      throw ex;
    }
    try {
      linear_acceleration_stddev = declare_parameter<double>("linear_acceleration_stddev", 0.0);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The linear_acceleration_stddev value provided was invalid");
      throw ex;
    }
    try {
      angular_velocity_stddev = declare_parameter<double>("angular_velocity_stddev", 0.0);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The angular_velocity_stddev value provided was invalid");
      throw ex;
    }
    try {
      orientation_stddev = declare_parameter<double>("orientation_stddev", 0.0);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The orientation_stddev value provided was invalid");
      throw ex;
    }

    uint32_t baud_rate{};
    auto fc = drivers::serial_driver::FlowControl::NONE;
    auto pt = drivers::serial_driver::Parity::NONE;
    auto sb = drivers::serial_driver::StopBits::ONE;

    try {
      m_device_name = declare_parameter<std::string>("device_name", "");
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try {
      baud_rate = declare_parameter<int>("baud_rate", 0);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

    try {
      const auto fc_string = declare_parameter<std::string>("flow_control", "");

      if (fc_string == "none") {
        fc = drivers::serial_driver::FlowControl::NONE;
      } else if (fc_string == "hardware") {
        fc = drivers::serial_driver::FlowControl::HARDWARE;
      } else if (fc_string == "software") {
        fc = drivers::serial_driver::FlowControl::SOFTWARE;
      } else {
        throw std::invalid_argument{
                "The flow_control parameter must be one of: none, software, or hardware."};
      }
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try {
      const auto pt_string = declare_parameter<std::string>("parity", "");

      if (pt_string == "none") {
        pt = drivers::serial_driver::Parity::NONE;
      } else if (pt_string == "odd") {
        pt = drivers::serial_driver::Parity::ODD;
      } else if (pt_string == "even") {
        pt = drivers::serial_driver::Parity::EVEN;
      } else {
        throw std::invalid_argument{
                "The parity parameter must be one of: none, odd, or even."};
      }
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try {
      const auto sb_string = declare_parameter<std::string>("stop_bits", "");

      if (sb_string == "1" || sb_string == "1.0") {
        sb = drivers::serial_driver::StopBits::ONE;
      } else if (sb_string == "1.5") {
        sb = drivers::serial_driver::StopBits::ONE_POINT_FIVE;
      } else if (sb_string == "2" || sb_string == "2.0") {
        sb = drivers::serial_driver::StopBits::TWO;
      } else {
        throw std::invalid_argument{
                "The stop_bits parameter must be one of: 1, 1.5, or 2."};
      }
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    m_device_config = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  }
  
  //private field variables
  std::unique_ptr<drivers::common::IoContext> m_owned_ctx{};
  std::string m_device_name{};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> m_device_config;
  std::unique_ptr<drivers::serial_driver::SerialDriver> m_serial_driver;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_pub;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;

  bool zero_orientation_set = false;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mpu6050SerialToImuNode>());
  rclcpp::shutdown();
  return 0;
}
