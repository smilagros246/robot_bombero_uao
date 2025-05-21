#pragma once
#include <termios.h>
#include <string>

class SerialPort {
public:
  SerialPort(const std::string &device, int baudrate);
  ~SerialPort();

  // Para mensajes de texto (opcional)
  std::string read_line(size_t max_bytes = 256) const;
  void write_line(const std::string &data) const;

  // Para mensajes binarios
  std::string read_fixed(size_t n_bytes) const;
  void write_bytes(const std::string &data) const;

private:
  int fd_;
};
