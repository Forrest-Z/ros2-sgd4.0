// Copyright 2022 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SGD_IO__SERIAL_HPP_
#define SGD_IO__SERIAL_HPP_

typedef signed char config_t;

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
#include <stdlib.h>
#include <string>
#include <stack>

#include <termios.h>

#include <unistd.h>
#include <unordered_map>

#include "io_exception.hpp"

namespace sgd_io
{

#define C_RAW 101
#define C_START_FRAME 102
#define C_STOP_FRAME 103
#define C_ESC_CHAR 104

class Serial
{

enum RECEIVER_STATE {
    INITIAL,
    ESTABLISH,
    AUTHENTIFICATE,
    WAIT_HEADER,
    IN_MSG,
    AFTER_ESC,
    ERROR,
    STOPPED = -1
};

private:
    // parameters    
    bool raw_;          // set raw mode
    char sframe_;       // start frame
    char stframe_;      // stop frame
    char esc_char_;     // escape character

    std::string read_buf_;
    std::string serial_msg_;

    int serial_port_;
    std::string logfile_;
    
    long time_at_start_;        // start time in millis
    std::fstream file_;

    std::unordered_map<config_t, char> config;

    RECEIVER_STATE rec_state_;
    
public:
    Serial();
    ~Serial();

    // use std::thread to read data from port??

    /**
     * @brief Open the defined port
     */
    void open_port(std::string port, int baud_rate);

    // methods to config serial
    bool set_raw(bool raw);
    bool set_stop_frame(char stop_frame);
    bool set_start_frame(char start_frame);
    bool set_esc_char(char esc_char);

    /**
     * @brief Close the serial port.
     */
    void close_port();

    /**
     * @brief Read a message (or part of it) from serial port
     * 
     * @return true if the end of message is reached
     */
    bool read_serial();

    /**
     * @brief Write string to serial.
     * 
     * @return the number of bytes written
     */
    int write_serial(std::string);

    std::string get_msg();
};

}

#endif