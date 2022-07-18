#include "sgd_io/serial.hpp"

namespace sgd_io
{

Serial::Serial()
{
    // set parameters to default values
    raw_ = false;        // raw mode
    sframe_ = '$';       // start frame
    stframe_ = '\n';     // stop frame
    esc_char_ = '#';     // escape character
}

Serial::~Serial() {}

void
Serial::open_port(std::string port, int baud_rate)
{
    if (rec_state_ > RECEIVER_STATE::ESTABLISH)
    {
        // connection already established
        //throw io_exception("Could not open connection to serial port. Connection already established.");
        //return;
    }

    struct termios tty_;
    serial_port_ = open(port.c_str(), O_RDWR);

    if (serial_port_ < 0)
    {
        rec_state_ = Serial::RECEIVER_STATE::STOPPED;
        throw io_exception("Error while opening port. Check all connections and try again.");
    }

    if (tcgetattr(serial_port_, &tty_) != 0)
    {
        rec_state_ = Serial::RECEIVER_STATE::STOPPED;
        throw io_exception("Error while opening port. Check all connections and try again.");
    }

// Control settings
    tty_.c_cflag &= ~CSIZE;
    tty_.c_cflag |= CS8;
    tty_.c_cflag &= ~CRTSCTS;
    tty_.c_cflag |= CLOCAL;

// Input settings
    tty_.c_iflag &= ~IXON;
    tty_.c_iflag |= IGNBRK;
    tty_.c_iflag &= ~BRKINT;
    tty_.c_iflag &= ~ICRNL;
    tty_.c_iflag &= ~IMAXBEL;

// Output settings
    tty_.c_oflag &= ~OPOST;
    tty_.c_oflag &= ~ONLCR;

// Local settings
    tty_.c_lflag &= ~ECHO;
    tty_.c_lflag &= ~ECHOE;
    tty_.c_lflag &= ~ECHOK;
    tty_.c_lflag &= ~ECHOCTL;
    tty_.c_lflag &= ~ECHOKE;
    tty_.c_lflag &= ~ICANON;
    tty_.c_lflag &= ~IEXTEN;
    tty_.c_lflag &= ~ISIG;
    tty_.c_lflag |= NOFLSH;

    tty_.c_cc[VTIME] = 0;
    tty_.c_cc[VMIN] = 0;

    switch (baud_rate)
    {
    case 9600:
        cfsetispeed(&tty_, B9600);
        cfsetospeed(&tty_, B9600);
        break;
    case 115200:
        cfsetispeed(&tty_, B115200);
        cfsetospeed(&tty_, B115200);
        break;
    default:
        cfsetispeed(&tty_, B9600);
        cfsetospeed(&tty_, B9600);
        break;
    }

    if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0)
    {
        rec_state_ = Serial::RECEIVER_STATE::STOPPED;
        throw io_exception("Error from tcsetattr: " + std::string(strerror(errno)));
    }
 
    rec_state_ = Serial::RECEIVER_STATE::WAIT_HEADER;
}

void
Serial::close_port() {
    close(serial_port_);
    rec_state_ = STOPPED;
}

bool
Serial::set_raw(bool raw)
{
    if (rec_state_ < RECEIVER_STATE::WAIT_HEADER)
    {
        raw_ = raw;
        return true;
    }
    return false;
}

bool
Serial::set_start_frame(char start_frame)
{
    // if (rec_state_ < RECEIVER_STATE::WAIT_HEADER)
    // {
    sframe_ = start_frame;
    return true;
    // }
    // return false;
}

bool
Serial::set_stop_frame(char stop_frame)
{
    // if (rec_state_ < RECEIVER_STATE::WAIT_HEADER)
    // {
    stframe_ = stop_frame;
    return true;
    // }
    // return false;
}

bool
Serial::set_esc_char(char esc_char)
{
    if (rec_state_ < RECEIVER_STATE::WAIT_HEADER)
    {
        esc_char_ = esc_char;
        return true;
    }
    return false;
}

bool
Serial::read_serial()
{
    // check if connection is established and we are authorized to read data
    if (rec_state_ < Serial::WAIT_HEADER)
    {
        return false;
    }
    
    // create empty array 
    char b [512];
    std::memset(&b, '\0', sizeof(b));

    // read one char per read
    int num_bytes;
    char c;
    int i = 0;
    while ((num_bytes = read(serial_port_, &c, 1)) > 0 && i < 512)  // catch i == 511
    {
        switch (rec_state_)
        {
        case WAIT_HEADER:   // message start received
            if (c == sframe_) {
                rec_state_ = Serial::IN_MSG;    // start/stop chars are not saved
            }
            break;

        case AFTER_ESC:     // simply save this char
            b[i++] = c;
            rec_state_ = Serial::IN_MSG;
            break;

        case IN_MSG:        // we are in the message
            if (c == stframe_) {    // stop frame received
                b[i] = '\0';
                read_buf_.append(b);        // append read chars to string
                serial_msg_ = read_buf_;    // save serial message so it can be accessed via get_msg method

                rec_state_ = raw_ ? RECEIVER_STATE::IN_MSG : RECEIVER_STATE::WAIT_HEADER;   // reset receiver state
                read_buf_.clear();
                
                c = '\0';
                return true;
            } else if (c == esc_char_ && !raw_) {
                rec_state_ = Serial::AFTER_ESC;
            } else {
                b[i++] = c;
            }
            break;

        default:
            // Error
            break;
        }
    }

    if (num_bytes < 0) {
        //RCLCPP_WARN(get_logger(), "Error reading %s", strerror(errno));
    }
    if (rec_state_ != WAIT_HEADER)
    {   // wir sind noch in der message aber es ist nichts mehr in serial
        read_buf_.append(b);
    }
    return false;
}

int
Serial::write_serial(const std::string msg_)
{
    if (rec_state_ < Serial::RECEIVER_STATE::WAIT_HEADER)
    {
        // port is not open or not configured
        throw io_exception("Could not write to port. Port is not open or not configured.");
    }

    unsigned char c[512];
    std::memset(&c, '\0', sizeof(c));

    uint16_t i;
    for (i = 0; i < msg_.length(); i++)
    {
        c[i] = msg_[i];
    }
    c[i++] = '\r';
    c[i++] = '\n';
    return write(serial_port_, c , i);
}

std::string
Serial::get_msg()
{
    std::string msg = serial_msg_.length() > 0 ? serial_msg_ : "";
    serial_msg_.clear();
    return msg;
}

}