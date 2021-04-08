
#include "../include/serial/serial.hpp"


int
read_line(std::ofstream *fout)
{
    char read_buf [256];

    std::memset(&read_buf, '\0', sizeof(read_buf));
    int num_bytes = read(serial_port_, &read_buf, sizeof(read_buf));

    if (num_bytes < 0) {
        std::cerr << "Error reading " << strerror(errno) << std::endl;
        return 1;
    }

    if (num_bytes <= 1) {return 0;}     // filter '\0' messages
    
    std::string line = read_buf;
    *fout << line;

    if (line.find("GPGLL", 0) < std::string::npos)
    {
        return -1;
    }
    return 0;
}

int
init_serial(const std::string port, const int baud)
{
    struct termios tty_;

    serial_port_ = open(port.c_str(), O_RDWR);
    isPortOpen_ = serial_port_ > 0 ? false : true;

    if (tcgetattr(serial_port_, &tty_) != 0)
    {
        std::cerr << "Could not open port " << port << std::endl;
        std::cerr << "Check all connections and try again." << std::endl;
        isPortOpen_ = false;
        return 1;       // TODO: Error handling
    }

    tty_.c_cflag &= ~PARENB;     // clear parity bit
    tty_.c_cflag &= ~CSTOPB;
    tty_.c_cflag &= ~CSIZE;
    //tty_.c_cflag != CS8;    // Compiler warning: statement has no effect
    tty_.c_cflag &= ~CRTSCTS;
    //tty_.c_cflag != (CREAD | CLOCAL);   // Compiler warning: statement has no effect

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

void
destroy(int signum)
{
    std::cout << "Terminate application" << std::endl;
    close(serial_port_);
    exit(0);
}

int main(int argc, char *argv[])
{
    // Parameter
    std::string port = "/dev/ttyACM0";
    int baud = 9600;

    std::cout << "Es wurden " << argc << " Argumente angegeben." << std::endl;
    
    for (int i = 1; i < argc; i++)
    {
        auto arg = argv[i];
        if (arg[0] == '-')
        {
            switch (arg[1])
            {
            case 'p':
                port = argv[++i];
                std::cout << "Port set to: " << port << std::endl;
                break;
            case 'b':
                baud = atoi(argv[++i]);
                std::cout << "Baud rate set to: " << baud << std::endl;
                break;
            default:
                std::cerr << "Invalid argument: " << argv[i] << std::endl;
                break;
            }
        } else
        {
            std::cout << "Unused arg: " << argv[i] << std::endl;
        }
    }

    if (init_serial(port, baud) > 0)
    {
        std::cerr << "Terminate programm" << std::endl;
        return 1;
    }
    
    signal(SIGINT, destroy);

    std::string path = "/tmp/serial";

    if (!fs::exists(path)) { fs::create_directories(path); }
    
    for (const auto & entry : fs::directory_iterator(path))
    {
        fs::remove(entry);
    }

    std::string tmp = fs::temp_directory_path().string() + "/serial/tmp.lck";
    int r;
    fs::path last_file;
    while (true)
    {
        r = 0;

        std::ofstream fout;
        fout.open(tmp,std::fstream::out);
        while (r == 0)
        {
            r = read_line(&fout);
        }
        fout.close();

        auto time = std::chrono::duration_cast<std::chrono::milliseconds>
                        (std::chrono::system_clock::now().time_since_epoch()).count();
        //std::string fname = "/tmp/serial/" + std::to_string(time) + ".nmea";
        
        fs::remove(last_file);
        fs::path fname(fs::temp_directory_path().string() + "/serial/" + std::to_string(time) + ".nmea");
        std::cout << "Write data to file " << fname << std::endl;
        fs::rename(tmp, fname);
        last_file = fname;
        //fs::permissions(fname, fs::perms::others_all, fs::perm_options::add);
        
        if (r > 0)
        {
            std::cout << "Error while reading serial port" << std::endl;
            break;
        }
    }
    close(serial_port_);

}