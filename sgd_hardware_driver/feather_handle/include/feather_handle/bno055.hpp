#ifndef SGD_HARDWARE__BNO055_HPP_
#define SGD_HARDWARE__BNO055_HPP_

#include <chrono>
#include <ctime>

#include <nlohmann/json.hpp>

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

} // namespace sgd_hardware_drivers

#endif