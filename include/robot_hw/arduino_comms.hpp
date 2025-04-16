#ifndef ROBOT_HW_ARDUINO_COMMS_HPP
#define ROBOT_HW_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

namespace robot_hw {

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{
public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
      std::cerr << "The ReadByte() call has timed out." << std::endl;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");
    size_t del_pos = response.find(" ");
    val_1 = std::atoi(response.substr(0, del_pos).c_str());
    val_2 = std::atoi(response.substr(del_pos + 1).c_str());
  }

  
  void read_imu_values(float &v1, float &v2, float &v3, float &v4, float &v5,
                      float &v6, float &v7, float &v8, float &v9, float &v10)
  {
      std::string response = send_msg("e\r");
  
      std::istringstream ss(response);
      std::string token;
      float values[10];
      int i = 0;
  
      while (std::getline(ss, token, ',') && i < 10) {
          try {
              values[i++] = std::stof(token);
          } catch (const std::exception &e) {
              std::cerr << "Conversion error: " << e.what() << " for token '" << token << "'\n";
              return;
          }
      }
  
      if (i != 10) {
          std::cerr << "Error: Expected 10 values but got " << i << std::endl;
          return;
      }
  
      v1 = values[0];
      v2 = values[1];
      v3 = values[2];
      v4 = values[3];
      v5 = values[4];
      v6 = values[5];
      v7 = values[6];
      v8 = values[7];
      v9 = values[8];
      v10 = values[9];
  }
  

  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_motor_values_ol(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "o " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

} // namespace robot_hw

#endif // ROBOT_HW_ARDUINO_COMMS_HPP
