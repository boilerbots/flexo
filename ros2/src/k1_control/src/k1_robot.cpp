
#include <cassert>
#include <mutex>
#include <iostream>
#include <numeric>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <linux/serial.h>
#include <rclcpp/logging.hpp>
#include "k1_control/k1_robot.h"
#include <sys/ioctl.h>

using namespace LibSerial;

namespace k1_control {

/*
* Pops the last element.
*/
static bool crc_check(std::vector<uint8_t> & data)
{
  uint8_t expected = data.back();
  data.pop_back();
  uint8_t checksum = std::accumulate(data.begin(), data.end(), 0);
  return checksum == expected;
}

static uint8_t add_crc(const std::span<const uint8_t> & data)
{
  return std::accumulate(data.begin(), data.end(), 0);
}

Robot::Robot(const std::string& device) :
  device_name_(device),
  logger_(rclcpp::get_logger("k1_robot")),
  running_(false) ,
  receive_message_count(0)
{

  conversion = COUNTS_PER_REV / (2 * M_PI);
  RCLCPP_INFO_STREAM(logger_, "Opening: " << device_name_);
#if 1
  serial_device_ = open(device_name_.c_str(), O_RDWR);
  // Set the serial port parameters
  termios tty;
  tcgetattr(serial_device_, &tty);
  //tty.c_cflag = B115200;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CLOCAL;
  tty.c_cflag |= CREAD;
  tty.c_cflag &= CSTOPB;
  tty.c_cflag |= CRTSCTS;
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~PARENB;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 1;
  cfsetspeed(&tty, B115200);
  if (tcsetattr(serial_device_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Error configuring port");
  }

  struct serial_rs485 rs485conf;

  /* Enable RS485 mode: */
  rs485conf.flags |= SER_RS485_ENABLED;

  /* Set logical level for RTS pin equal to 1 when sending: */
  //rs485conf.flags |= SER_RS485_RTS_ON_SEND;
  /* or, set logical level for RTS pin equal to 0 when sending: */
  rs485conf.flags &= ~(SER_RS485_RTS_ON_SEND);

  /* Set logical level for RTS pin equal to 1 after sending: */
  rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;
  /* or, set logical level for RTS pin equal to 0 after sending: */
  //rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

  /* Set rts delay before send, if needed: */
  rs485conf.delay_rts_before_send = 0;

  /* Set rts delay after send, if needed: */
  rs485conf.delay_rts_after_send = 0;

  /* Set this flag if you want to receive data even while sending data */
  //rs485conf.flags |= SER_RS485_RX_DURING_TX;

  if (ioctl (serial_device_, TIOCSRS485, &rs485conf) < 0) {
    RCLCPP_ERROR(logger_, "Failed to set RS485 mode");
  }


#else
  serial_device_.Open(device_name_); // device_name_ doesn't work
  serial_device_.SetBaudRate( BaudRate::BAUD_115200 );
  serial_device_.SetFlowControl( FlowControl::FLOW_CONTROL_HARDWARE );
  serial_device_.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );
  serial_device_.SetStopBits( StopBits::STOP_BITS_1 );
  serial_device_.SetParity( Parity::PARITY_NONE );
#endif

  RCLCPP_INFO_STREAM(logger_, "Start receiving Thread");
  t = std::thread(&Robot::receive, this);
}

Robot::~Robot() {
  stopRobot();
  running_ = false;
#if 0
  serial_device_.Close();
#else
  close(serial_device_);
#endif
}

void Robot::receive()
{
  State state = IDLE;
  uint8_t next_byte;
  uint8_t address;
  uint8_t expected_data_len;
  uint8_t data_len;
  std::vector<uint8_t> data;

  RCLCPP_INFO_STREAM(logger_, "Start receiving Loop");
  running_ = true;
  while(running_) {
    //RCLCPP_INFO(logger_, "running");
#if 0
    try {
      serial_device_.ReadByte(next_byte, 1000);
    } catch (LibSerial::ReadTimeout & e) {
      continue;
    }
#else
    if (read(serial_device_, &next_byte, 1) < 1) {
      continue;
    }
#endif

    RCLCPP_INFO_STREAM(logger_, "state=" << state << " received: " << std::hex << int(next_byte) << std::dec);
    switch (state)
    {
      case IDLE:
        if (next_byte == 0xFF) {
          state = START1;
        }
      break;
      case START1:
        if (next_byte == 0xFF) {
          state = START2;
        } else {
          state = IDLE;  // start over
        }
      break;
      case START2:
        address = next_byte;
        data.clear();
        data.push_back(address);
        state = ADDRESS;
      break;
      case ADDRESS:
        expected_data_len = next_byte;
        data.push_back(expected_data_len);
        data_len = 0;
        state = DATA;
      break;
      case DATA:
        data.push_back(next_byte);
        ++data_len;
        if (data_len == expected_data_len) {
          state = IDLE;
          if (!crc_check(data)) {
            RCLCPP_ERROR(logger_, "Bad CRC");
            continue;
          }
          if (data[2] == 0xA3) {
            uint16_t position = (data[3] * 256) + data[4] + CORRECTION;
            RCLCPP_INFO(logger_, "channel %d  position %d", int(address), int(position));
            hw_lock.lock();
            current_position[address] = position;
            ++receive_message_count;
            hw_lock.unlock();
          } 
        }
      break;
    }
  }
}

#if 0
k1_robot::RobotState Robot::readOnce() {
  std::lock_guard<std::mutex> lock(control_mutex_);
  if (!active_control_) {
    current_state_ = robot_->readOnce();
  } else {
    current_state_ = readOnceActiveControl();
  }
  return current_state_;
}
#endif

void Robot::send(const std::span<const uint8_t> & data) {
  std::vector<uint8_t> msg = {255, 255};
  msg.insert(msg.end(), data.begin(), data.end());
  msg.push_back(add_crc(data));
  RCLCPP_INFO(logger_, "sending: ");
#if 0
  for (auto d : msg) {
    //RCLCPP_INFO(logger_, "%x ", d);
    serial_device_.WriteByte(d);
  }
  serial_device_.FlushOutputBuffer();
#else
  write(serial_device_, msg.data(), msg.size());
  tcdrain(serial_device_);
#endif
}

void Robot::get_position() {
  int last_receive_count;
  std::vector<uint8_t> local_position(position.begin(), position.end());
  for (int channel = 0; channel < REAL_CHANNELS; channel++) {
    local_position[0] = channel;
    std::span<const uint8_t> msg(local_position);
    last_receive_count = receive_message_count;
    send(msg);
    usleep(10000);
    while (last_receive_count == receive_message_count) {
      RCLCPP_WARN(logger_, "waiting for position");
      usleep(1000000);
      send(msg);
    }
  }
}

void Robot::compute_hwval() {
  hw_lock.lock();
  for (int ch = 0; ch < REAL_CHANNELS; ++ch) {
    hw_val[ch * 2] = current_position[ch] / 256;
    hw_val[ch * 2 + 1] = current_position[ch] % 256;
  }
  hw_lock.unlock();
}

void Robot::stopRobot() {
}

void Robot::startRobot() {
  RCLCPP_INFO_STREAM(logger_, "Starting the robot");
  send(std::span<const uint8_t>(initialize));
  usleep(300000);
  send(std::span<const uint8_t>(enable));
  usleep(2000);
  for (int xx = 0; xx < 4; ++xx) {
    get_position();
  }
  compute_hwval();
}

void Robot::writeJointPositions(const std::vector<double>& position) {
  //std::lock_guard<std::mutex> lock(control_mutex_);
  hw_lock.lock();
  for (int xx = 0; xx < REAL_CHANNELS; ++xx) {
    current_position[xx] = home_position[xx] - (position[xx] * conversion);
  }
  hw_lock.unlock();
}

#if 0
k1_control::RobotState Robot::readOnceActiveControl() {
  // When controller is active use active control to read the robot state
  const auto [current_state, _] = active_control_->readOnce();
  return current_state;
}
#endif

}
