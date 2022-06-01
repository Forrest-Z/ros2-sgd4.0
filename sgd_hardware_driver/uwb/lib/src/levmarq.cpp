#include "levmarq.hpp"

namespace sgd_hardware_drivers
{

LevMarq::LevMarq()
{
    last_pose(0) = 0.0;
    last_pose(1) = 0.0;
}

std::pair<double, double>
LevMarq::estimatePose()
{
	//std::cout << "Received " << measuredRanges.size() << " ranges." << std::endl;
    LMFunctor functor;
    functor.tags = tags;
	functor.measuredRanges = measuredRanges;
	//functor.m = measuredRanges.size();

    // 'x' is vector of length 'n' containing the initial values for the parameters.
	// The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
	// The LM optimization inputs should not be confused with the x input values.
    Eigen::VectorXf x(2);
    x(0) = last_pose(0);
    x(1) = last_pose(1);

	Eigen::LevenbergMarquardt<LMFunctor, float> lm(functor);
	lm.minimize(x);
	//std::cout << "LM optimization status: " << status << std::endl;
    measuredRanges.clear();
	//
	// Results
	// The 'x' vector also contains the results of the optimization.
	//
	//std::cout << "Optimization results" << std::endl;
	//std::cout << "\txT: " << x(0) << std::endl;
	//std::cout << "\tyT: " << x(1) << std::endl;

    last_pose(0) = x(0);
    last_pose(1) = x(1);

    return {x(0), x(1)};
}

}