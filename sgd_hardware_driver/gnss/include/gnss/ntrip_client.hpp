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

#include <ctype.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <ctime>
#include <cstring>
#include <string>

#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <poll.h>

#include <iostream>
#include <mutex>
#include <memory>
#include <thread>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

#define MAXDATASIZE 1000 /* max number of bytes we can get at once */

namespace sgd_hardware_drivers
{

extern std::mutex rtcmMutex;
extern char rtcm[1000];
extern int numbytes;
extern std::mutex ggaMutex;
extern std::string gga;

struct ntrip_opts
{
    std::string server;         // server address
    std::string port;           // port number
    std::string mountpnt;       // mountpoint
    std::string auth;           // authentification string

    bool nmea = false;          // send gga to server
};

class Ntrip_Client
{

private:
    const std::string crlf = "\r\n";

    ntrip_opts options_;    // ntrip client options
    std::thread::native_handle_type thread_handle;  // thread handle to end thread
    
    int sockfd = 0;     // socket file descriptor
    uint8_t err_;       // error marker

    std::time_t send_nmea = 0;  // timestamp from last sent nmea message

    /**
     * @brief Try to get data from ntrip caster
     */
    void update();

public:
    Ntrip_Client(ntrip_opts options);
    ~Ntrip_Client();

    /**
     * @brief Start ntrip client
     */
    void start_client();
};

} // namespace sgd_hardware_drivers