#include "sgd_global_planner/path_smoothing.hpp"


using std::vector;
using std::string;

namespace sgd_algorithms
{
    path_smoothing::path_smoothing()
    {

    }

    path_smoothing::~path_smoothing()
    {
        
    }

    /*bool 
    path_smoothing::initialize()
    {

    }*/

    /**
     * @brief Gets the path to be smoothen from an .csv file and returns it as a vector of vectors.
     * 
     * @return The path in a 2-D vector form.
     */
    vector<vector<double>>
    get_path(string file_name)
    {
		vector<vector<double>> path;
		vector<double> row;
		string line, word;
		
		// Open the file using ‘fstream’ C++ library
		std::fstream file (file_name, std::ios::in);

		if(file.is_open())
		{
		// Read the file line by line using the getline() method as each line ends with a newline character
		// and store the next line of the file stream in the string variable.
			while(getline(file, line))
			{
				row.clear();
				// Extract the words that are separated in the line by a comma. 
				std::stringstream str(line);
				//Reads the data only till the next comma sign in a single line.
				//Then store each word in a vector named row
				while(getline(str, word, ','))
					row.push_back(stod(word));
				// Store each row in the 2-D vector named path.
				path.push_back(row);
			}
		}
		else
			std::cout<<"Could not open the file\n";

        return path;
    }

    /**
     * @brief It takes a path as an input and returns a smoothened version of it.
     * 
     * @param old_path 
     * @param weight_data 
     * @param weight_smooth 
     * @param tolerance 
     * @return std::vector<std::vector<double>> 
     */
    vector<vector<double>>
    smoothen_path(vector<vector<double>> path, double weight_data, double weight_smooth, double tolerance)
    {
        // Create a copy of the original path 
        vector<vector<double>> new_path(path);
        // Get rows in the 2d vector
        int dims = path[0].size();
        double change = tolerance;
        double x_i, y_i, y_prev, y_next, y_i_saved;

        while(change >= tolerance)
            {
                change = 0.0;
                for(int i = 1; i < (new_path.size() - 1); i++)
                    for(int j = 0; j < dims; j++)
                        {
                            x_i = path.at(i).at(j);
                            y_i = new_path.at(i).at(j);
                            y_prev = new_path.at(i - 1).at(j);
                            y_next = new_path.at(i + 1).at(j);
                            y_i_saved = y_i;
                            y_i += (weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i)));
                            new_path[i][j] = y_i;
                            change += std::abs(y_i - y_i_saved);
                        }
            }

        return new_path;
    }

    /**
     * @brief Interpolates 2 points
     * 
     * @param x0 X coordinate of point 1
     * @param y0 Y coordinate of point 1
     * @param x1 X coordinate of point 2
     * @param y1 Y coordinate of point 2
     * @return vector<double> Interpolated point
     */
    vector<double> interpolate(double x0, double y0, double x1, double y1)
{
	double xp, yp;
	xp = (x0 + x1) / 2;
 	yp = y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
	vector<double> point3{xp, yp};
	return point3;
}

    /**
     * @brief Uses interpolation to increase the number of points between paths
     * 
     * @return vector<vector<double>> New, extended path
     */
    vector<vector<double>>
    increase_points(vector<vector<double>> path)
    {
        int length_list = ((path.size()) * 2) - 2;
        vector<double> point2add(2);
        vector<vector<int>> spaced_path( length_list , vector<int> (path.[0].size(), 0));
        int i = 0;

        for(int j = 0; j < length_list; j++)
        {
            if((j % 2) == 0)
            {
                spaced_path.at(j) = path.at(i);
                i++;
            }
            else{
                point2add = interpolate(path.at(i - 1).at(0), path.at(i - 1).at(1), path.at(i).at(0), path.at(i).at(1));
                spaced_path.at(j).at(0) = point2add.at(0);
                spaced_path.at(j).at(1) = point2add.at(1);
            }
        }
        
        return spaced_path;
    }

}

