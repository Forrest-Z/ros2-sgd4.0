#pragma once

#include <string>
#include <variant>
#include <unordered_map>

class IGPS_Message
{
public:
    virtual ~IGPS_Message() = default;

    //! \brief Parse received message.
    virtual void parse_msg(std::string msg) = 0;

    //! \brief Returns timestamp of last valid gps message.
    double time() {
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

    int get_data_int(std::string param)
    {
        return data_int_.find(param) != data_int_.end() ? data_int_.at(param) : 0;
    }

    double get_data_double(std::string param)
    {
        return data_double_.find(param) != data_double_.end() ? data_double_.at(param) : 0.0;
    }

    std::string get_data_string(std::string param)
    {
        return data_string_.find(param) != data_string_.end() ? data_string_.at(param) : "";
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

    void add_param(std::string name, std::variant<std::string, int, double> value) 
    {
        if (name.length() == 0) {return;}
    
        if (std::holds_alternative<std::string>(value))
        {
            data_string_.insert(std::make_pair(name, std::get<std::string>(value)));
        }
        else if (std::holds_alternative<int>(value))
        {
            data_int_.insert(std::make_pair(name, std::get<int>(value)));
        }
        else
        {
            data_double_.insert(std::make_pair(name, std::get<double>(value)));
        }
    }

};
