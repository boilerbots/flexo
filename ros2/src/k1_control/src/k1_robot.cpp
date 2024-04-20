
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

  float default_conversion = COUNTS_PER_REV / (2 * M_PI);
  conversion[0] = -default_conversion; // l thigh
  conversion[1] = default_conversion;  // l knee
  conversion[2] = default_conversion;  // l calf
  conversion[3] = default_conversion;  // r thigh
  conversion[4] = -default_conversion;  // r knee
  conversion[5] = -default_conversion;  // r calf
  conversion[6] = default_conversion;  // l shoulder
  conversion[7] = -default_conversion; // r shoulder
  conversion[8] = default_conversion;  // l hip
  conversion[9] = -default_conversion;  // l foot
  conversion[10] = -default_conversion;  // r hip
  conversion[11] = default_conversion;  // r foot
  conversion[12] = default_conversion;  // r ankle
  conversion[13] = -default_conversion; // left lower
  conversion[14] = -default_conversion;  // r arm
  conversion[15] = default_conversion;  // r hand
  conversion[16] = default_conversion;
  RCLCPP_INFO_STREAM(logger_, "Opening: " << device_name_);
  serial_device_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_device_ < 0) {
    RCLCPP_ERROR(logger_, "Error open serial device ");
  }
  // Set the serial port parameters
  termios tty;
  tcgetattr(serial_device_, &tty);
  //tty.c_cflag = B115200;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CLOCAL;
  tty.c_cflag |= CREAD;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CRTSCTS;
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~PARENB;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 0;
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

  /* Disable NONBLOCK to make life easy */
  int flags = fcntl(serial_device_, F_GETFL);
  flags &= ~O_NONBLOCK;
  fcntl(serial_device_, F_SETFL, flags);

  RCLCPP_INFO_STREAM(logger_, "Start receiving Thread");
  t = std::thread(&Robot::receive, this);
}

Robot::~Robot() {
  stopRobot();
  running_ = false;
  close(serial_device_);
}

void Robot::receive()
{
  State state = IDLE;
  uint8_t next_byte;
  uint8_t address;
  uint8_t expected_data_len;
  uint8_t data_len = 0;
  std::vector<uint8_t> data;
  int read_count;

  RCLCPP_INFO(logger_, "Start receiving Loop");
  running_ = true;
  while(running_) {
    read_count = read(serial_device_, &next_byte, 1);
    if (read_count < 1) {
      //RCLCPP_INFO(logger_, "NO DATA");
      continue;
    }

    //RCLCPP_INFO(logger_, "read_count=%d", read_count);
    //RCLCPP_INFO_STREAM(logger_, "state=" << state << " read_count=" << read_count << " received: " << std::hex << int(next_byte) << std::dec);
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
  RCLCPP_WARN_STREAM(logger_, "\nReceive Thread Terminated\n");
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
  //RCLCPP_INFO(logger_, "sending: ");
#if 0
  for (auto d : msg) {
    RCLCPP_INFO(logger_, " %x", int(d));
  }
#endif
  uint8_t *dptr = msg.data();
  int byte_count = msg.size();
  ssize_t sent; 
  while (byte_count) {
    sent = write(serial_device_, dptr, byte_count);
    dptr += sent;
    byte_count -= sent;
  }
  tcdrain(serial_device_);
}

void Robot::get_position() {
  int last_receive_count;
  std::vector<uint8_t> local_position(read_position.begin(), read_position.end());
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
  for (int xx = 0; xx < 4; ++xx) {
    get_position();
  }
  compute_hwval();
  send(std::span<const uint8_t>(enable));
  usleep(2000);
}

void Robot::writeJointPositions(const std::vector<double>& position) {
  //std::lock_guard<std::mutex> lock(control_mutex_);
  hw_lock.lock();
  for (int xx = 0; xx < REAL_CHANNELS; ++xx) {
#if 0
    if (xx == 16) {
      RCLCPP_WARN(logger_, "ch=%d  pos=%lf", xx, position[xx]);
    }
#endif
    current_position[xx] = home_position[xx] - (position[xx] * conversion[xx]);
  }
  hw_lock.unlock();
  compute_hwval();
  std::vector<uint8_t> local_position(set_all_position.begin(), set_all_position.end());
  local_position.insert(local_position.end(), hw_val.begin(), hw_val.end());
  std::span<const uint8_t> msg(local_position);
  send(std::span<const uint8_t>(msg));
}

void Robot::readJointPositions(std::vector<double>& position) {
  hw_lock.lock();
  for (int xx = 0; xx < REAL_CHANNELS; ++xx) {
    //current_position[xx] = home_position[xx] - (position[xx] * conversion);
    position[xx] = (home_position[xx] - current_position[xx]) / conversion[xx];
  }
  hw_lock.unlock();
}
}
