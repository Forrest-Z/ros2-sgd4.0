#ifndef  PATH_SMOOTHING_HPP_
#define  PATH_SMOOTHING_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <sstream>
#include <iomanip>

#include "sgd_util/geotools.hpp"

class path_smoothing
{
    
    private:
        double weight;
        double weight_data;
        double weight_smooth;
        double tolerance;
        int path_increase;

        /**
         * @brief It uses interpolation to increment the number of points in the path. 
         * This improves the end result as there ar more points to smoothen the path.
         * 
         * @param original_path     The path to be extended by interpolation.
         * @return vector<sgd_util::LatLon> 
         */
        vector<sgd_util::LatLon> extend_path(vector<sgd_util::LatLon> original_path);
        

    public:
        // Constructor
        /**
         * @brief Construct a new path smoothing object.
         * The weight values must add 1.
         * 
         * @param weight_data       Changes the weight or importance that is given to the original data.
         * @param weight_smooth     A higher value prioritizes how smooth the path should be.
         * @param tolerance         It states how much change per iteration is necessary to keep iterating.
         * @param path_increase     A factor by which the number of points in the original path will be increased through interpolation.
         */
        path_smoothing(double weight_data_, double weight_smooth_, double tolerance_, int path_increase_);

        /**
         * @brief Construct a new path smoothing object
         * If no parameters are specified, they are set to default values.
         */
        path_smoothing();


        // Public methods

        /**
         * @brief It uses the gradient ascent algorithm to make the trajectory smoother.
         * 
         * @return vector<sgd_util::LatLon>  Returns a smoothened version of the oiginal path.
         */
        vector<sgd_util::LatLon> smoothen_path(vector<sgd_util::LatLon> path);

        // Setters
        /**
         * @brief From 1 to 9, how smooth do you want the path to be, being 9 the smoothest.
         * This function simplifies the assigment of weights so only one parameter is needed.
         * Both weights can be independently chosen by using the constructor.
         * 
         * @param weight 
         */
        void set_weights(double weight);

        /**
         * @brief Set how much the number of points in the path will increase.
         * 
         * @param path_increase_ 
         */
        void set_path_increase(int path_increase_);

        // Getters

        /**
         * @brief Get the weights: weight_data and weight_smooth used by the smoothing method.
         * 
         * @return std::tuple<double, double> 
         */
        std::tuple<double, double> get_weights();

        /**
         * @brief Get the path increase parameter
         * 
         * @return int 
         */
        int get_path_increase();
        
    protected:


};


#endif