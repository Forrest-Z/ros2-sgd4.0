#include "imu/imu_bno055.hpp"

namespace sgd_sensors
{

using namespace std::chrono_literals;   // if a timer is used

IMU_BNO055::IMU_BNO055():
    nav2_util::LifecycleNode("imu_bno055", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("config_mode", rclcpp::ParameterValue(false));
    double var[3] = {0.0,0.0,0.0};
    add_parameter("acc_var", rclcpp::ParameterValue(var));
    add_parameter("mag_var", rclcpp::ParameterValue(var));
    add_parameter("gyr_var", rclcpp::ParameterValue(var));

}

IMU_BNO055::~IMU_BNO055()
{
    // Destroy
}

nav2_util::CallbackReturn
IMU_BNO055::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    cov_count = 0;
    mean_acc = Vector3();
    mean_gyr = Vector3();
    mean_hea = Vector3();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
IMU_BNO055::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    imu_pub_->on_activate();
    temp_pub_->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
IMU_BNO055::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");

    imu_pub_->on_deactivate();
    temp_pub_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
IMU_BNO055::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    imu_pub_.reset();
    temp_pub_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
IMU_BNO055::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
IMU_BNO055::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("config_mode", config_mode_);
}

void
IMU_BNO055::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init publisher and subscriber");

    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);
    serial_sub_ = create_subscription<sgd_msgs::msg::Serial>(
        serial_topic, default_qos, std::bind(&IMU_BNO055::on_serial_received, this, std::placeholders::_1));

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", default_qos);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu_temp", default_qos);
}

void
IMU_BNO055::on_serial_received(const sgd_msgs::msg::Serial::SharedPtr msg)
{
    if (msg->msg.find("IMU") == std::string::npos) return;     // message is not from imu sensor

    int time;
    std::regex time_regex("IMU:(\\d*)");
    std::smatch time_match;
    if (std::regex_search(msg->msg, time_match, time_regex))
    {
        time = std::stoi(time_match[1]);
    }

    std::regex data_reg("(\\w):\\{([-\\d\\.,]*)\\}");
    //std::smatch data_match;
    auto begin = std::sregex_iterator(msg->msg.begin(), msg->msg.end(), data_reg);
    auto end = std::sregex_iterator();

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now();
    imu_msg.header.frame_id = "imu";

    for (std::sregex_iterator i = begin; i != end; i++) {
        std::smatch match = *i;
        const char *c = match[1].str().c_str();

        switch (*c)
        {
        case 'A':   // linear acceleration
            imu_msg.linear_acceleration = data_to_vector(match[2]);
            break;

        case 'H':   // heading in quaternions
            imu_msg.orientation = data_to_quat(match[2]);
            break;

        case 'G':   // gyroscope
            imu_msg.angular_velocity = data_to_vector(match[2]);
            break;
        
        case 'C':   // calibration 0-255
            {
                u_int16_t cal = std::stoi(match[2]);
                //sys_calibration = floor(cal/pow(2,6));    // system calibration from bno055
                acc_calibration = floor((cal % 64)/16);
                hea_calibration = floor((cal % 16)/4);
                gyr_calibration = floor(cal % 4);
                system_calibrated = (cal >= 255);
            }
            break;
        case 'T':
            //RCLCPP_INFO(get_logger(), "Set temperature.");
            return;
        default:
            RCLCPP_WARN(get_logger(), "Unknown data type: %s", *c);
            break;
        }
    }
    
    if (std::min(acc_calibration, std::min(hea_calibration, gyr_calibration)) < 2 && !system_calibrated)
    {
        if (now().seconds() > last_calib_msg_.seconds() + 2)
        {
            RCLCPP_INFO(get_logger(), "BNO055 not calibrated! Status is A: %i, M: %i, G: %i",
                acc_calibration, hea_calibration, gyr_calibration);
            last_calib_msg_ = now();
        }
        return;
    }
    
    tf2::Quaternion q(
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z,
        imu_msg.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(get_logger(), "IMU heading %.2f", yaw);

    if (config_mode_ && now().seconds() > (last_calib_msg_.seconds() + 2))
    {
        if (cov_count < 1) RCLCPP_INFO(get_logger(), "Start calibration");
        
        // Calculate mean values
        if (cov_count < max_cov_count)
        {
            Vector3 euler = quat_to_euler(imu_msg.orientation);
            meas_acc[cov_count] = Vector3(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
            meas_hea[cov_count] = euler;
            meas_gyr[cov_count] = Vector3(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

            cov_count++;

            mean_acc.x += imu_msg.linear_acceleration.x / max_cov_count;
            mean_acc.y += imu_msg.linear_acceleration.y / max_cov_count;
            mean_acc.z += imu_msg.linear_acceleration.z / max_cov_count;
            
            mean_hea.x += euler.x / max_cov_count;
            mean_hea.y += euler.y / max_cov_count;
            mean_hea.z += euler.z / max_cov_count;
            
            mean_gyr.x += imu_msg.angular_velocity.x / max_cov_count;
            mean_gyr.y += imu_msg.angular_velocity.y / max_cov_count;
            mean_gyr.z += imu_msg.angular_velocity.z / max_cov_count;
        } 
        else
        {
            // calculate covarianze matrizes
            cov_acc = calc_cov_matrix(meas_acc, mean_acc);
            cov_hea = calc_cov_matrix(meas_hea, mean_hea);
            cov_gyr = calc_cov_matrix(meas_gyr, mean_gyr);

            config_mode_ = false;
            RCLCPP_INFO(get_logger(), "Calibration done!");
        }
    } else {
        imu_msg.linear_acceleration_covariance = cov_acc;
        imu_msg.orientation_covariance = cov_hea;
        imu_msg.angular_velocity_covariance = cov_gyr;
        imu_pub_->publish(imu_msg);
    }
}

geometry_msgs::msg::Vector3
IMU_BNO055::data_to_vector(std::string data)
{
    std::regex reg("-?[\\d\\.]+");
    std::smatch data_match;
    auto begin = std::sregex_iterator(data.begin(), data.end(), reg);
    auto end = std::sregex_iterator();

    int k = 0;
    geometry_msgs::msg::Vector3 vec;
    for (std::sregex_iterator i = begin; i != end; ++i) {
        std::smatch match = *i;

        double d = 0.0;
        try
        {
            d = std::stod(match.str());
        }
        catch(const std::invalid_argument& e)
        {
            RCLCPP_WARN(get_logger(), "Parsing data %s returned error %s", match.str().c_str(), e.what());
            geometry_msgs::msg::Vector3 v;
            return v;       // returns 0 vector on error
        }
        
        switch (k)
        {
        case 0:
            vec.x = d;
            break;
        case 1:
            vec.y = d;
            break;
        case 2:
            vec.z = d;
            break;
        default:
            break;
        }
        k++;
    }
    return vec;
}

geometry_msgs::msg::Quaternion
IMU_BNO055::data_to_quat(std::string data)
{
    std::regex reg("-?[\\d\\.]+");
    std::smatch data_match;
    auto begin = std::sregex_iterator(data.begin(), data.end(), reg);
    auto end = std::sregex_iterator();

    int k = 0;
    geometry_msgs::msg::Quaternion quat;
    for (std::sregex_iterator i = begin; i != end; ++i) {
        std::smatch match = *i;

        double d = 0.0;
        try
        {
            d = std::stod(match.str());
        }
        catch(const std::invalid_argument& e)
        {
            RCLCPP_WARN(get_logger(), "Parsing data %s returned error %s", match.str().c_str(), e.what());
            geometry_msgs::msg::Quaternion v;
            return v;       // returns 0 vector on error
        }
        
        switch (k)
        {
        case 0:
            quat.w = d;
            break;
        case 1:
            quat.x = d;
            break;
        case 2:
            quat.y = d;
            break;
        case 3:
            quat.z = d;
            break;
        default:
            break;
        }
        k++;
    }
    return quat;
}

std::array<double, 9UL>
IMU_BNO055::calc_cov_matrix(Vector3 meas[], Vector3 mean)
{
    std::array<double, 9UL> cov_matrix;
    // calculate covarianze matrizes
    for (int8_t i = 0; i < max_cov_count; i++)
    {
        // Varianzen
        cov_matrix[0] += pow(meas[i].x - mean.x, 2.0);
        cov_matrix[4] += pow(meas[i].y - mean.y, 2.0);
        cov_matrix[8] += pow(meas[i].z - mean.z, 2.0);

        // Covarianzen
        cov_matrix[1] += (meas[i].x - mean.x) * (meas[i].y - mean.y);
        cov_matrix[2] += (meas[i].x - mean.x) * (meas[i].z - mean.z);
        cov_matrix[5] += (meas[i].y - mean.y) * (meas[i].z - mean.z);
    }
    
    // Matrizes are symmetric
    cov_matrix[3] = cov_matrix[1];
    cov_matrix[6] = cov_matrix[2];
    cov_matrix[7] = cov_matrix[5];

    for (int8_t j = 0; j < 9; j++)
    {
        cov_matrix[j] /= (max_cov_count - 1);
    }
    return cov_matrix;
}

IMU_BNO055::Vector3
IMU_BNO055::quat_to_euler(geometry_msgs::msg::Quaternion q)
{
    Vector3 e;
    e.x = atan2(2*(q.w * q.x + q.y * q.z), 1-2*(q.x * q.x + q.y * q.y));
    e.y = asin(2*(q.w * q.y - q.z * q.x));
    e.z = atan2(2*(q.w * q.z + q.x * q.y), 1-2*(q.y * q.y + q.z * q.z));
    return e;
}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_sensors::IMU_BNO055>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
