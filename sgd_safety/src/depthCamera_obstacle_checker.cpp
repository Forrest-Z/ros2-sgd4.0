#include "sgd_safety/depthCamera_obstacle_checker.hpp"


namespace sgd_safety
{
    DepthCameraObstacleChecker::DepthCameraObstacleChecker()
    {
        init_val = 1;
    }

    DepthCameraObstacleChecker::~DepthCameraObstacleChecker()
    {
        init_val = 0;
        p.stop();
    }

    std::tuple<char, double> 
    DepthCameraObstacleChecker::get_depthcam_speed(void)
    {
        switch (init_val)
        {
            // Values not initialized
        case 0:
            retcode = 'i'; // Initialization error
            speed = 0.0;
            break;

            // First program execution. Start camera
        case 1:
            p.start();          // Configure and start the pipeline
            init_val++;
            retcode = 's';      // Camera has been started
            // Purposefully not using break, so it goes to the next case.

            // Get frames from camera and calculate calculate the robot speed
        case 2:
            retcode = 's';  // Speed has been calculated
            speed = this->compute_speed();
            break;
        
        default:
            retcode = 'e'; // Unexpected error
            speed = 0.0;
            break;
        }
        return std::make_tuple(retcode, speed);
    }

    double 
    DepthCameraObstacleChecker::compute_speed(void)
    {
        // Blocks program until frames arrive
        rs2::frameset frames = p.wait_for_frames();
        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();
        // Get the depth frame's dimensions
        auto frame_width = depth.get_width();
        auto frame_height = depth.get_height() - 1;

        
        float dist, prev_dist = 0, obstacle_dist = 0, slope;
        int obstacle_detect_local = 0;
        int obstacle_detect_global = 0;
        bool obstacle_detect_line = false;
                
        for(int j = detection_width / -2; j <= detection_width / 2; j++)
        {
            for(int i = 0; i <= detection_distance; i++)
            {
                dist = depth.get_distance((int)((frame_width / 2) + j), (int)(frame_height - i));

                // Discard noise
                if(dist == 0)
                    break;

                // Checks if the distance measured is expected: if so, resets obstacle counter; otherwise marks it as an obstacle indication on that line (local).
                slope = (dist - prev_dist) / ((i == 0) ? 1 : i-(i-1));
                
                if(slope < 0)  
                    obstacle_detect_local++;
                else
                    obstacle_detect_local = 0; 

                // If there are certain number of obstacle indications, it adds it to the global count.
                if(obstacle_detect_local > obstacle_sensitivity_local)
                {
                    obstacle_detect_local = 0;                    
                    obstacle_detect_global++;
                    obstacle_detect_line = true;
                    break;
                }
                prev_dist = dist;   
            }
            
            // The obstacles must be contiguous, otherwise their indications are discarded.
            if (obstacle_detect_line == false)
                obstacle_detect_global == 0;
            else
                obstacle_detect_line = false;


            // If this count exceeds certain number, it means that the obstacle has been corroborated on multiple lines, so its presence is declared.
            if(obstacle_detect_global >= obstacle_sensitivity_global)
            {
                obstacle_dist = dist;
                break;
            }
            else
                obstacle_dist = 0;
        }

        if(obstacle_dist > depthCamera_max_dist)
        speed = 1.0;
        else if(obstacle_dist < depthCamera_min_dist)
            speed = 0.0;
        else
            speed = (double)((obstacle_dist - depthCamera_min_dist) / (depthCamera_max_dist - depthCamera_min_dist)); 

        return speed;

    }

    char
    DepthCameraObstacleChecker::stop_depth_camera(void)
    {
        if(init_val >= 1)
        {
            p.stop();
            retcode = 'p';  // camera stopped
        }
        else
            retcode = 'n';  // camera not initialized

        return retcode;
    }

    char 
    DepthCameraObstacleChecker::get_retcode(void)
    {
        return retcode;
    }

    
}

