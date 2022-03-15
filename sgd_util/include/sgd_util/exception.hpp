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
        msg_ = description;
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