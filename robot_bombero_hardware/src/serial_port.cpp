#include "robot_bombero_hardware/serial_port.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <stdexcept>
#include <iostream>

SerialPort::SerialPort(const std::string &device, int baudrate) {
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    throw std::runtime_error("Error al abrir puerto serial: " + std::string(strerror(errno)));
  }

  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    throw std::runtime_error("Error de tcgetattr: " + std::string(strerror(errno)));
  }

  cfmakeraw(&tty);  // Importante para comunicación binaria sin interferencias

  cfsetispeed(&tty, baudrate);
  cfsetospeed(&tty, baudrate);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;  // Timeout en décimas de segundo

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("Error de tcsetattr: " + std::string(strerror(errno)));
  }
}

SerialPort::~SerialPort() {
  if (fd_ > 0) {
    close(fd_);
  }
}

void SerialPort::write_line(const std::string &data) const {
  ::write(fd_, data.c_str(), data.size());
}

std::string SerialPort::read_line(size_t max_bytes) const {
  char buffer[256];
  memset(buffer, 0, max_bytes);
  int num_bytes = ::read(fd_, buffer, max_bytes);
  if (num_bytes < 0) {
    throw std::runtime_error("Error de lectura: " + std::string(strerror(errno)));
  }
  return std::string(buffer, num_bytes);
}

void SerialPort::write_bytes(const std::string &data) const {
  ssize_t written = ::write(fd_, data.data(), data.size());
  if (written < 0) {
    throw std::runtime_error("Error al escribir en el puerto serial: " + std::string(strerror(errno)));
  }
}

std::string SerialPort::read_fixed(size_t n_bytes) const {
  std::string buffer(n_bytes, '\0');
  size_t total_read = 0;

  while (total_read < n_bytes) {
    ssize_t n = ::read(fd_, &buffer[total_read], n_bytes - total_read);
    if (n < 0) {
      throw std::runtime_error("Error de lectura serial: " + std::string(strerror(errno)));
    }
    if (n == 0) break;  // EOF o timeout
    total_read += n;
  }

  buffer.resize(total_read);
  return buffer;
}
