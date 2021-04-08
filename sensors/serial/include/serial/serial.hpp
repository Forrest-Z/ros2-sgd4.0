
#ifndef SERIAL_TO_ROS_HPP_
#define SERIAL_TO_ROS_HPP_

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>

#include <iostream>

#include <memory>

#include <signal.h>
#include <stdio.h>
#include <string>

#include <termios.h>

#include <unistd.h>


namespace fs = std::filesystem;

int serial_port_;
bool isPortOpen_;


//! \brief Open serial port and configure for further operations.
//! \param port port specifier
//! \param baud baud-rate
int init_serial(const char port, const int baud);

//! \brief Read line
int read_line(std::ofstream *fout);

void destroy();


#endif