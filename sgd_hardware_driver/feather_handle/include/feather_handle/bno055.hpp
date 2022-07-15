#ifndef SGD_HARDWARE__BNO055_HPP_
#define SGD_HARDWARE__BNO055_HPP_

#include <chrono>
#include <ctime>

#include <nlohmann/json.hpp>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_hardware_drivers
{

class BNO055
{

struct imu_data
{
    uint8_t cal;
    double data[3];
};

private:
    uint8_t cal_[4];    // system, acc, gyro, mag
    double acc_[3];
    double gyr_[3];
    double eul_[3];
    double mag_[3];
    int8_t temp_;
    uint64_t millis_ = 0;

    void parse_calib(uint8_t cal);
    void str_to_vec3(nlohmann::json json, double *vec);
public:
    BNO055();
    ~BNO055();

    /**
     * @brief Parse the json string message.
     * 
     * @param msg 
     * @return 1 for a message (acc, etc.), 2 for b message (temp) and 0 if message type is unknown
     */
    int parse_msg(std::string msg);

    /**
     * @brief Get the acceleration
     * 
     * @return imu_data 
     */
    imu_data get_acc();

    /**
     * @brief Get the gyroscope data
     * 
     * @return imu_data 
     */
    imu_data get_gyro();

    /**
     * @brief Get the magnet object
     * 
     * @return imu_data 
     */
    imu_data get_magnet();

    /**
     * @brief Get the euler angles in degrees
     * 
     * @return imu_data 
     */
    imu_data get_euler();

    /**
     * @brief Millis as received from microcontroller
     * 
     * @return long 
     */
    uint64_t get_millis();
};

BNO055::BNO055() {}
BNO055::~BNO055() {}

int
BNO055::parse_msg(std::string msg)
{
    // check if temp or acc message
    auto js = nlohmann::json::parse(msg);
    auto& s_id = js["s_id"];

    if (s_id == "bno055a")
    {
        // message contains acceleration, gyro, heading, etc.
        if (js.count("time"))   millis_ = js["time"].get<uint64_t>();
        if (js.count("cal"))    parse_calib(js["cal"].get<uint8_t>());
        if (js.count("acc"))    str_to_vec3(js["acc"], acc_);
        if (js.count("gyr"))    str_to_vec3(js["gyr"], gyr_);
        if (js.count("euler"))  str_to_vec3(js["euler"], eul_);
        if (js.count("mag"))    str_to_vec3(js["mag"], mag_);
        return 1;
    }
    else if (s_id == "bno055b")
    {
        // message contains temperature
        if (js.count("temp"))    temp_ = js["temp"].get<int8_t>();
        return 2;
    }
    return 0;
}

BNO055::imu_data
BNO055::get_acc()
{
    imu_data d;
    d.cal = cal_[1];
    std::copy(std::begin(acc_), std::end(acc_), std::begin(d.data));
    return d;
}

BNO055::imu_data
BNO055::get_gyro()
{
    imu_data d;
    d.cal = cal_[2];
    std::copy(std::begin(gyr_), std::end(gyr_), std::begin(d.data));
    return d;
}

BNO055::imu_data
BNO055::get_magnet()
{
    imu_data d;
    d.cal = cal_[2];
    std::copy(std::begin(mag_), std::end(mag_), std::begin(d.data));
    return d;
}

BNO055::imu_data
BNO055::get_euler()
{
    imu_data d;
    d.cal = cal_[3];
    // Transform euler from north = 0° to ENU (east = 0°) coordinate frame
    double heading = (eul_[2] - 90.0) * -1.0;
    eul_[2] = heading < -180.0 ? heading + 360.0 : heading;

    for (int i = 0; i < 3; i++)
    {
        d.data[i] = eul_[i] / 180.0 * M_PI;
    }

    return d;
}

uint64_t
BNO055::get_millis()
{
    return millis_;
}

void
BNO055::str_to_vec3(nlohmann::json json, double *vec)
{
    for (int i = 0; i < 3; i++)
    {
        vec[i] = json[i].get<double>();
    }
}

void
BNO055::parse_calib(uint8_t cal)
{
    cal_[3] = floor((cal % 16)/4);  // magnetometer
    cal_[2] = floor(cal % 4);       // gyro
    cal_[1] = floor((cal % 64)/16); // acc
    cal_[0] = (cal >= 255);         // system
}

} // namespace sgd_hardware_drivers

#endif