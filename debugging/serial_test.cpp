
#include <cassert>
#include <mutex>
#include <iostream>
#include <numeric>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "serial_test.h"

using namespace std;


/*
* Pops the last element.
*/
static bool crc_check(std::vector<uint8_t> & data)
{
  uint8_t expected = data.back();
  data.pop_back();
  uint8_t checksum = std::accumulate(data.begin(), data.end(), 0);
  //cout << "checksum=" << int(checksum) << " expected=" << int(expected) << endl;
  return checksum == expected;
}

static uint8_t add_crc(const std::span<const uint8_t> & data)
{
  return std::accumulate(data.begin(), data.end(), 0);
}

Robot::Robot(const std::string& device) :
  device_name_(device),
  running_(false) ,
  receive_message_count(0)
{
  cout << "Opening: " << device_name_ << endl;
  serial_device_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_device_ < 0) {
    cerr << "ERROR opening serial device" << endl;
    exit(1);
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
    cout << "Error configuring port\n";
  }
  cout << "setup\n";

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
    cout << "Failed to set RS485 mode\n";
  }

  tcflush(serial_device_, TCIOFLUSH);
  cout << "flushed\n";

  /* Disable NONBLOCK to make life easy */
  int flags = fcntl(serial_device_, F_GETFL);
  flags &= ~O_NONBLOCK;
  fcntl(serial_device_, F_SETFL, flags);

  cout << "Start receiving Thread\n";
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
  int receive_byte_count;

  cout << "Start receiving Loop\n";
  running_ = true;
  while(running_) {
    receive_byte_count = read(serial_device_, &next_byte, 1);
    if (receive_byte_count < 1) {
      cout << "No Data\n";
      continue;
    }

    //cout << "state=" << state << "  count=" << receive_byte_count << " received: " << hex << int(next_byte) << " \n";
    switch (state)
    {
      case IDLE:
        if (next_byte == 0xff) {
          state = START1;
        }
      break;
      case START1:
        if (next_byte == 0xff) {
          state = START2;
        } else {
          state = IDLE; // start over
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
            cout << "Bad CRC\n";
            continue;
          }
          if (data[2] == 0xA3) {
            uint16_t position = (data[3] * 256) + data[4] + CORRECTION;
            cout << "channel=" << int(address) << " position=" << int(position) << endl;
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
#if 0
  cout << "sending: len=" << msg.size();
  for (auto d : msg) {
    cout << " " << hex << int(d);
  }
  cout << endl;
#endif
  // write(serial_device_, msg.data(), msg.size());
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
  std::vector<uint8_t> local_position(position.begin(), position.end());
  for (int channel = 0; channel < REAL_CHANNELS; channel++) {
    local_position[0] = channel;
    std::span<const uint8_t> msg(local_position);
    last_receive_count = receive_message_count;
    send(msg);
    usleep(5000);
    while (last_receive_count == receive_message_count) {
      cout << "waiting for position\n";
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
  cout << "Starting the robot\n";
  send(std::span<const uint8_t>(initialize));
  usleep(300000);
  send(std::span<const uint8_t>(enable));
  usleep(2000);
  for (int xx = 0; xx < 10; ++xx) {
    get_position();
  }
  compute_hwval();
}

#if 0
k1_control::RobotState Robot::readOnceActiveControl() {
  // When controller is active use active control to read the robot state
  const auto [current_state, _] = active_control_->readOnce();
  return current_state;
}
#endif

int main()
{
  Robot robot("/dev/serial0");
  robot.startRobot();
  while(true) {
    usleep(1000000);
  }
}
