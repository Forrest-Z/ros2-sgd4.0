#include <stdio.h>
#include <string>
#include <iostream>

#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <termios.h>
#include <unistd.h>

using namespace std;

class navilock_ublox6_parser
{
public:
    navilock_ublox6_parser();
    navilock_ublox6_parser(int const baud_rate);
    ~navilock_ublox6_parser();

    bool isPortOpen_;
    char readLine();

private:
    
    int const baud_rate_;
    char const port_[80];
    struct termios tty_;
    int serial_port_;

    int init();
};

navilock_ublox6_parser::navilock_ublox6_parser():
    isPortOpen_(false), baud_rate_(9600), port_("/dev/ttyACM0") {
        init();
    }

navilock_ublox6_parser::navilock_ublox6_parser(int baud_rate):
    isPortOpen_(false), baud_rate_(baud_rate), port_("/dev/ttyACM0") {
        init();
    }

navilock_ublox6_parser::~navilock_ublox6_parser() {
    close(serial_port_);
}

int navilock_ublox6_parser::init() {
    serial_port_ = open(port_, O_RDWR);
    isPortOpen_ = serial_port_ > 0 ? false : true;

    if (tcgetattr(serial_port_, &tty_) != 0)
    {
        printf("Error %i from togetattr %s\n", errno, strerror(errno));
        return 1;       // TODO: Error handling
    }

    tty_.c_cflag &= ~PARENB;     // clear parity bit
    tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag != CS8;
    tty_.c_cflag &= ~CRTSCTS;
    tty_.c_cflag != CREAD | CLOCAL;

    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~ECHO;
    tty_.c_lflag &= ~ECHOE;
    tty_.c_lflag &= ~ECHONL;
    tty_.c_lflag &= ~ISIG;

    tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty_.c_oflag &= ~OPOST;
    tty_.c_oflag &= ~ONLCR;

    tty_.c_cc[VTIME] = 10;
    tty_.c_cc[VMIN] = 0;

    cfsetispeed(&tty_, B9600);      // TODO: use user defined baud-rate
    cfsetospeed(&tty_, B9600);

    return 0;
}

char navilock_ublox6_parser::readLine() {
    char read_buf [256];
    std::memset(&read_buf, '\0', sizeof(read_buf));
    int num_bytes = read(serial_port_, &read_buf, sizeof(read_buf));
    
    if (num_bytes < 0)
    {
        printf("Error reading %s", strerror(errno));        // TODO: error handling
        return 1;
    }
    printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
    return *read_buf;
}


int main(void)
{
    navilock_ublox6_parser* parser = new navilock_ublox6_parser(9600);  // Methode 1
    

    for (size_t i = 0; i < 100; i++)
    {
        cout << parser->readLine() << endl;
    }
    
    

    // ####### Todo parse #########

    /*
    Sensor_msgs/NavSatFix.msg

    NavSatStatus status;
    float64 latitude;
    float64 longitude;
    float64 altitude;

    float64[9] position_covariance;
    uint8 position_covariance_type; -> {0;1;2;3}


    */


    delete parser;
    return 0;
}