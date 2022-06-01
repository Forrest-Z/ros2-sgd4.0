#ifndef NAV_SGD__PATH_SMOOTHING_HPP_
#define NAV_SGD__PATH_SMOOTHING_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <sstream>
#include <iomanip>

#include "sgd_util/geotools.hpp"

namespace sgd_ctrl
{

    typedef std::pair<double, double> xy_pnt;

    class PathSmoothing
    {
    private:
        double weight;
        double weight_data;
        double weight_smooth;
        double tolerance;
        double interpolation_resolution_;

        /**
         * @brief It uses interpolation to increment the number of points in the path.
         * This improves the end result as there ar more points to smoothen the path.
         *
         * @param original_path     The path to be extended by interpolation.
         * @return vector<xy_pnt>
         */
        std::vector<xy_pnt> extend_path(std::vector<xy_pnt> original_path);

        /**
         * @brief Interpolate between two points in local coordinate frame. 
         *
         * @param pt1 the first point
         * @param pt2 the second point
         * @param max_distance maximum allowed distance between points
         * @return the vector containing only the new points, starting from the first point
         */
        std::vector<xy_pnt> interpolate(xy_pnt pt1, xy_pnt pt2, double max_distance = 1.0);

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
        PathSmoothing(double weight_data_, double weight_smooth_, double tolerance_, double interpolation_resolution_);

        /**
         * @brief Construct a new path smoothing object
         * If no parameters are specified, they are set to default values.
         */
        PathSmoothing();

        /**
         * @brief Destroy the Path Smoothing object
         */
        ~PathSmoothing();

        // Public methods

        /**
         * @brief It uses the gradient ascent algorithm to make the trajectory smoother.
         *
         * @return vector<sgd_util::LatLon>  Returns a smoothened version of the oiginal path.
         */
        std::vector<xy_pnt> smoothen_path(std::vector<xy_pnt> path);

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
        void set_interpolation_resolution(double interpolation_resolution);

        // Getters

        /**
         * @brief Get the weights: weight_data and weight_smooth used by the smoothing method.
         *
         * @return xy_pnt
         */
        xy_pnt get_weights();

        /**
         * @brief Get the path increase parameter
         *
         * @return int
         */
        double get_interpolation_resolution();

    };

} // namespace nav_sgd

#endif