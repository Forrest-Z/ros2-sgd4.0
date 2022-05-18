#ifndef SGD_HARDWARE__WH_FCRUISER_HPP_
#define SGD_HARDWARE__WH_FCRUISER_HPP_

#include <nlohmann/json.hpp>

namespace sgd_hardware_drivers
{

#define RIGHT_WHEEL_FACTOR 57.93
#define LEFT_WHEEL_FACTOR 59.00 

class WH_FCruiser
{
private:
    ulong millis_ = 0;
    float volt_ = 37.0;
    float temp_ = 20.0;

    double vel_lin_x_ = 0.0, vel_ang_z_ = 0.0;
    double pos_x_ = 0.0, pos_y_ = 0.0, ori_z_ = 0.0;

    // parameters
    double wheel_circ_;     // wheel circumference
    double wheel_sep_;      // wheel separation

public:
    WH_FCruiser(double wheel_circ, double wheel_sep);
    ~WH_FCruiser();

    /**
     * @brief Parse the json message
     * 
     * @param msg 
     */
    void parse_msg(std::string msg);

    /**
     * @brief Get the position relative to the start position
     * 
     * @param pos_x 
     * @param pos_y 
     */
    void get_position(double &pos_x, double &pos_y);

    /**
     * @brief Get the relative orientation
     * 
     * @return double 
     */
    double get_orientation();

    /**
     * @brief Get the linear velocity
     * 
     * @return double 
     */
    double get_linear_vel();

    /**
     * @brief Get the angular velocity
     * 
     * @return double 
     */
    double get_angular_vel();

    /**
     * @brief Get the battery voltage
     * 
     * @return float 
     */
    float get_batt_voltage();

    /**
     * @brief Get the measured temperature of the controller board
     * 
     * @return float 
     */
    float get_temp();

    /**
     * @brief Translates the ROS cmd_vel message to the controller specific message
     * that can be send to the serial port
     * 
     * @param vel_lin_x linear velocity in x direction
     * @param vel_ang_z angular velocity around z axis
     * @return std::string the message
     */
    std::string cmd_vel(double vel_lin_x, double vel_ang_z);

    inline int sign(double x)
    {
        return (x < 0.0 ? -1 : (x > 0.0));
    }
};

WH_FCruiser::WH_FCruiser(double wheel_circ, double wheel_sep)
{
    wheel_circ_ = wheel_circ;
    wheel_sep_ = wheel_sep;
}

WH_FCruiser::~WH_FCruiser()
{
}

void
WH_FCruiser::parse_msg(std::string msg)
{
    // msg: {"s_id": "hover", "time": 123, "Rm": 0, "Lm": 0, "volt": 0, "temp": 0}
    try
    {
        auto js = nlohmann::json::parse(msg);
        auto& s_id = js["s_id"];

        if (s_id == "hover")
        {
            // save values to temporary variable
            ulong tmp_time_;
            double tmp_rm_, tmp_lm_;

            if (js.count("time"))   tmp_time_ = js["time"].get<ulong>();
            if (js.count("Rm"))     tmp_rm_ = js["Rm"].get<int>();
            if (js.count("Lm"))     tmp_lm_ = js["Lm"].get<int>();
            if (js.count("volt"))   volt_ = (2 * volt_ + js["volt"].get<int>()/100.0) / 3.0;
            if (js.count("temp"))   temp_ = js["temp"].get<int>()/10.0;
            
            if (millis_ == 0)   millis_ = tmp_time_;

            // calculate current position
            tmp_rm_ /= RIGHT_WHEEL_FACTOR;
            tmp_lm_ /= LEFT_WHEEL_FACTOR;

            vel_lin_x_ = ((tmp_rm_ + tmp_lm_) * wheel_circ_) / 2;
            vel_ang_z_ = ((tmp_rm_ - tmp_lm_) * wheel_circ_) / wheel_sep_ * 1.14;
            
            // set orientation
            ori_z_ += vel_ang_z_ * (tmp_time_ - millis_) / 1000.0;

            pos_x_ = pos_x_ + vel_lin_x_ * (tmp_time_ - millis_) / 1000.0 * cos(ori_z_);
            pos_y_ = pos_y_ - vel_lin_x_ * (tmp_time_ - millis_) / 1000.0 * sin(ori_z_);

            millis_ = tmp_time_;
        }
    }
    catch(const nlohmann::json::exception& e)
    {
        std::cerr << "JSON parse error. JSON string: " << msg << ", error: " << e.what() << std::endl;
    }   
}

void
WH_FCruiser::get_position(double &pos_x, double &pos_y)
{
    pos_x = pos_x_;
    pos_y = pos_y_;
}

double
WH_FCruiser::get_orientation()
{
    return ori_z_;
}

double
WH_FCruiser::get_linear_vel()
{
    return vel_lin_x_;
}

double
WH_FCruiser::get_angular_vel()
{
    return vel_ang_z_;
}

float
WH_FCruiser::get_batt_voltage()
{
    return volt_;
}

float
WH_FCruiser::get_temp()
{
    return temp_;
}

std::string
WH_FCruiser::cmd_vel(double vel_lin_x, double vel_ang_z)
{
    // max motor speed 1000
    double r = (vel_lin_x + vel_ang_z/2) * 250;
    double l = (-vel_ang_z + (vel_lin_x + vel_ang_z/2)) * 250;
    
    l += sign(l)*50;
    r += sign(r)*50;
    // send: right, left
    return std::to_string((int)round(r)) + "," + std::to_string((int)round(l));
}

} // namespace sgd_hardware_drivers


#endif