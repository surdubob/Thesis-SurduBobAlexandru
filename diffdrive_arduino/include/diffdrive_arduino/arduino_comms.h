#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <serial/serial.h>
#include <cstring>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
class ArduinoComms
{


public:

  ArduinoComms()
  {  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
      //: serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
  {  }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { 
    // return serial_conn_.isOpen();
    return socket_fd != 0; 
  }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  serial::Serial serial_conn_;  ///< Underlying serial connection 

  int socket_fd = 0;
  char buffer[1024] = { 0 };
  int status, valread;
  struct sockaddr_in serv_addr;


  int connect_to_socket() {
    
    if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }
  
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8082);
  
    // Convert IPv4 and IPv6 addresses from text to binary
    // form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)
        <= 0) {
        printf(
            "\nInvalid address/ Address not supported \n");
        return -1;
    }
  
    if ((status
         = connect(socket_fd, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }
    return 1;
  }

  std::string send_socket_command(const char *command) {
    send(socket_fd, command, strlen(command), 0);
    valread = read(socket_fd, buffer, 1024);
    return std::string(buffer);
  }
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H