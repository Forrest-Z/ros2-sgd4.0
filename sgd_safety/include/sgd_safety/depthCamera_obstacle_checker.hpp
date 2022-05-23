#ifndef SGD_SAFETY__DEPTH_CAMERA_OBSTACLE_CHECKER_HPP_
#define SGD_SAFETY__DEPTH_CAMERA_OBSTACLE_CHECKER_HPP_

#include <librealsense2/rs.hpp>
#include <tuple>


namespace sgd_safety
{

class DepthCameraObstacleChecker
{
private:
    // Parameters
    const int obstacle_sensitivity_local = 5;       // Minimum ocurrences per line to be considered an obstacle
    const int obstacle_sensitivity_global = 3;      // Minimum occurrences of obstacle_sensitivity_local to raise a detection flag.
    const int detection_distance = 130;             // How far (in pixels) will the objects be detected
    const int detection_width = 20;                 // This value should be an odd number.
    double robot_width_;         // Width of the robot in m including the wheels
    double distance_min_;        // Minimal distance the robot has to keep with respect to obstacles
    double distance_max_;        // At any distance smaller than this, the robot should start decreasing its speed.
    const float width_tolerance = 0.1;      // A tolerance on the robot width in m for an extra safety factor.
    rs2::pipeline p;    // Create a Pipeline
    rs2::frameset frames;
    uint8_t init_val = 0;
    char retcode = 'u'; // uninitialized return code
    double compute_speed(void);
    double speed = 0.0;

public:
    DepthCameraObstacleChecker();
    ~DepthCameraObstacleChecker();
     std::tuple<char, double> get_depthcam_speed(void);
    char stop_depth_camera(void);
    char get_retcode(void);

};

} 

#endif