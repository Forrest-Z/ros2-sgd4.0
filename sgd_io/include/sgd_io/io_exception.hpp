#ifndef SGD_IO__SERIAL_EXCEPTION_HPP_
#define SGD_IO__SERIAL_EXCEPTION_HPP_

#include <exception>
#include <string>

namespace sgd_io
{

class io_exception : std::exception
{
private:
    std::string msg_;
public:
    io_exception(std::string msg) {
        msg_ = msg;
    }

    const char * what() const throw () {
        return msg_.c_str();
    }
};

}

#endif