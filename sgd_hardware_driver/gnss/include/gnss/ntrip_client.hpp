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

#include <iostream>
#include <mutex>
#include <memory>
#include <thread>

#define MAXDATASIZE 1000 /* max number of bytes we can get at once */

namespace sgd_hardware_drivers
{

extern std::mutex rtcmMutex;
extern std::string rtcm;
extern std::mutex ggaMutex;
extern std::string gga;

struct ntrip_opts
{
    std::string server;
    const char *port;
    std::string mountpnt;
    std::string user;
    std::string password;

    //const char *nmea;
    bool nmea = false;
};

class Ntrip_Client
{

private:
    const std::string crlf = "\r\n";

    ntrip_opts options_;
    std::thread t;
    uint8_t err_;

    int sockfd = 0;
    char nmeabuffer[200] = "$GPGGA,"; /* our start string */
    size_t nmeabufpos = 0;
    size_t nmeastarpos = 0;
    
    // variables for sending nmea message
    std::time_t send_nmea = 0;
    //std::string gga_msg_ = "";

    const char encodingTable [64] = {
        'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
        'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
        'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
        'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
    };

    /**
     * @brief Encode username and password.
     * 
     * @return std::string encoded string
     */
    std::string encode_str();

    /**
     * @brief Try to get data from ntrip caster
     * @return Error code
     */
    void update();

public:
    Ntrip_Client(ntrip_opts options);
    ~Ntrip_Client();

    void get_sourcetable();

    void start_client();
};

} // namespace sgd_hardware_drivers