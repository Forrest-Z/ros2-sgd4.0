#ifndef SGD_UTIL__EXCEPTION_HPP_
#define SGD_UTIL__EXCEPTION_HPP_

#include <exception>
#include <string>

namespace sgd_util
{

//! \brief Exception for xml error
class XmlError : public std::exception
{
public:
   
    XmlError(std::string description) {
        msg_ = "";
    }

    //! \brief Return the error description if provided, otherwise an empty string.
    std::string ErrorStr() {
        return msg_;
    }

    const char * what() const throw () {
        return msg_.c_str();
    }

private:
    std::string msg_;
};

}   // namespace sgd_util

#endif  // SGD_UTIL__EXCEPTION_HPP_