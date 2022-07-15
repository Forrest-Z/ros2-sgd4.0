#ifndef SGD_HARDWARE__FRSKY_RX8R_HPP_
#define SGD_HARDWARE__FRSKY_RX8R_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <sstream>

namespace sgd_hardware_drivers
{

class FrSky_RX8R
{

struct LIGHT
{
    uint8_t mode;
    uint8_t rgb[3] = {0,0,0};
};

private:
    double max_vel_, max_rot_;
    double vel_x_, rot_z_;

    bool is_master_switch_pressed_;
    bool last_msg_equ_zero_;
    
    // left and right light mode
    //uint8_t modes[2] = {0,0};
    //uint8_t rgb[2][3] = {{0,0,0}, {0,0,0}};

    LIGHT left_, right_;
    bool has_error_;
    std::string error_;
public:
    FrSky_RX8R();
    FrSky_RX8R(double max_vel, double max_rot);
    ~FrSky_RX8R();

    /**
     * @brief Parse the message
     * 
     * @param msg 
     */
    void parse_msg(std::string msg);

    /**
     * @brief Compute velocity and rotation from 
     * 
     * @param ch1 channel for velocity in x direction
     * @param ch2 channel for rotation around z axis
     * @param ch_m channel for master switch
     */
    void set_vel_rot(int ch1, int ch2, int ch_m);

    /**
     * @brief Return the velocity and rotation
     * 
     * @return std::pair<double, double> 
     */
    std::pair<double, double> get_vel_rot();

    /**
     * @brief Set the lights object
     * 
     * @param light the light to change
     * @param ch received value from receiver
     */
    void set_lights(LIGHT& light, int ch);

    /**
     * @brief Returns true if master switch is pressed
     * 
     * @return Returns true if master switch is pressed
     */
    bool is_in_master_mode();

};

FrSky_RX8R::FrSky_RX8R() : max_vel_(0.5), max_rot_(0.5) {}

FrSky_RX8R::FrSky_RX8R(double max_vel, double max_rot)
    : max_vel_(max_vel), max_rot_(max_rot) {}

FrSky_RX8R::~FrSky_RX8R() {}

void
FrSky_RX8R::parse_msg(std::string msg)
{
    if (msg.find("RX8R") == std::string::npos)
    {
        // message has to start with 'RX8R' to be sure it is from the remote control
        return;
    }

    // 8 channels in message, each channel ranges from 172 to 1811
    // channels 1 and 2 are for movement
    // channel 3 controls lights on left side, channel 6 lights on right side
    // channel 8 is master switch
    std::string parsed;
    std::stringstream input_stringstream(msg);
    int channels[8];
    int i = 0;

    // parse channels
    while (getline(input_stringstream, parsed, ',') && i < 8)
    {
        // message starts with 'RX8R' so skip this part
        if (parsed.find("RX8R") != std::string::npos)
        {
            continue;
        }

        try
        {
            // parse value and add it to array
            channels[i] = std::stoi(parsed);
            i++;
        }
        catch(const std::invalid_argument& e)
        {
            error_ = "Could not parse value " + parsed + ": " + e.what();
        }
    }

    set_vel_rot(channels[0], channels[1], channels[7]);
    set_lights(left_, channels[2]);
    set_lights(right_, channels[5]);
}

void
FrSky_RX8R::set_vel_rot(int ch1, int ch2, int ch_m)
{
    // subtract 992 so each channel ranges from -820 to +819
    ch1 -= 992;
    ch2 -= 992;
    ch_m -= 992;

    // if value is <10 set velocity to 0 to avoid shaking of the shared guide dog
    // calculate speed from channel input
    vel_x_ = abs(ch1) < 10 ? 0 : (double)ch1 / 820.0 * max_vel_;
    rot_z_ = abs(ch2) < 10 ? 0 : (double)-ch2 / 820.0 * max_rot_;

    // publish velocity command
    is_master_switch_pressed_ = ch_m > 0;
}

std::pair<double, double>
FrSky_RX8R::get_vel_rot()
{
    return {vel_x_, rot_z_};
}

void
FrSky_RX8R::set_lights(LIGHT& light, int ch)
{
    // subtract 992 so each channel ranges from -820 to +819
    ch -= 992;

    // set state to 0, 1 or 2
    light.mode = abs(ch) < 20 ? 2 : ch > 0;
    // set color depending on mode
    switch (light.mode)
    {
    case 0:
        light.rgb[0] = 255;
        break;
    
    case 1:
        light.rgb[0] = 255;
        light.rgb[1] = 165;
        break;

    case 2:
        light.rgb[2] = 255;
        break;
    default:
        break;
    }
}

bool
FrSky_RX8R::is_in_master_mode()
{
    return is_master_switch_pressed_;
}
    
} // namespace sgd_hardware_drivers

#endif