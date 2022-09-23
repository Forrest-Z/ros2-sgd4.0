// Copyright 2021 HAW Hamburg
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

#ifndef SGD_HARDWARE_DRIVERS__INMEA_MESSAGE_HPP_
#define SGD_HARDWARE_DRIVERS__INMEA_MESSAGE_HPP_

#pragma once

#include <string>
#include <variant>
#include <unordered_map>
#include <vector>
#include <variant>

class Error
{
public:
    enum ERROR {
        NO_ERROR = 0,
        INVALID_ARG,
        XML_PARSE_ERROR
    };

    Error(ERROR error_id) : id_(error_id) {
        msg_ = "";
    }

    Error(ERROR error_id, std::string description) : id_(error_id), msg_(description) {
        // empty constructor
    }

    //! \brief Return the error id.
    int ErrorID() {
        return id_;
    }

    //! \brief Return the error description if provided, otherwise an empty string.
    std::string ErrorStr() {
        return msg_;
    }

    //! \brief Create a message containing the error id and description.
    std::string to_string() {
        std::string s = "Error ID: " + std::to_string(id_) + "\n";
        s.append("Description: " + msg_);
        return s;
    }

private:
    int id_;
    std::string msg_;
};

class INMEA_Message
{

public:
    virtual ~INMEA_Message() = default;

    //! \brief Import xml file
    //! \return Error code
    virtual int import_xml(std::string xml_file) = 0;

    //! \brief Parse received message.
    //! \return Error code
    virtual int parse_msg(std::string msg) = 0;

    //! \brief Return true if all required values were received.
    virtual bool msg_complete() = 0;

    //! \brief Returns timestamp of last valid gps message.
    double gps_time() {
        return data_double_.find("time") != data_double_.end() ? data_double_.at("time") : 0;
    }

    //! \brief Returns latitude of last valid gps message.
    double latitude() {
        return data_double_.find("latitude") != data_double_.end() ? data_double_.at("latitude") : 0;
    }

    //! \brief Returns longitude of last valid gps message.
    double longitude() {
        return data_double_.find("longitude") != data_double_.end() ? data_double_.at("longitude") : 0;
    }

    //! \brief Returns true if gps signal is fixed.
    int fix() {
        return data_int_.find("fix") != data_int_.end() ? data_int_.at("fix") : 0;
    }
    
    //! \brief Tries to get the specified parameter from saved data.
    //! \returns Pair containing variant holding the data and an integer for easy access with std::get<int>(variant).
    //! If the requested parameter is not available, return -1 for integer value
    std::pair<std::variant<int, double, std::string>, int> get_data(std::string param)
    {
        if (data_double_.find(param) != data_double_.end())
        {
            return {data_double_.at(param), 1};
        } else if (data_int_.find(param) != data_int_.end())
        {
            return {data_int_.at(param), 1};
        } else if (data_string_.find(param) != data_string_.end())
        {
            return {data_string_.at(param), 1};
        }
        return {"", -1};
    }

    //! \brief Return true if an error occured.
    bool has_error() {
        return errors.size() > 0;
    }

    //! \brief Return the number of errors.
    int error_count() {
        return errors.size();
    }

    //! \brief Return the last error.
    Error get_last_error() {
        if (errors.size() < 1)
        {
            return Error(Error::NO_ERROR);
        } else {
            return errors.back();
        }
    }

    //! \brief Clear all data.
    void clear() {
        data_string_.clear();
        data_int_.clear();
        data_double_.clear();
    }

protected:
    std::unordered_map<std::string, std::string> data_string_;
    std::unordered_map<std::string, double> data_double_;
    std::unordered_map<std::string, int> data_int_;

    std::vector<Error> errors;

    //! \brief Add data from message with type string
    //! \return true if the insertion was successful otherwise false
    bool add_data(std::string name, std::variant<int, double, std::string> value)
    {
        if (name.length() == 0)
        {
            return false;
        }

        switch (value.index())
        {
        case 0:     // int
            return data_int_.insert({name, std::get<int>(value)}).second;
            break;

        case 1:     // double
            return data_double_.insert({name, std::get<double>(value)}).second;
            break;

        case 2:     // string
            return data_string_.insert({name, std::get<std::string>(value)}).second;
            break;

        default:
            break;
        }
        return false;
    }

    //! \brief Return the number of saved values.
    int num_values() {
        return data_int_.size() + data_double_.size() + data_string_.size();
    }
};

#endif  // SGD_HARDWARE_DRIVERS__INMEA_MESSAGE_HPP_