#pragma once
#ifdef MAA_DEVEL
#include "sgd_odometry.h"
#else
#include "../../../motorcontroller/include/motorcontroller/sgd_odometry.h"
#endif // MAA_DEVEL
#include <vector>
#include <algorithm>

#define SGD_UWB_BEACON_ID_TYPE uint16_t
/// <summary>
/// A very basic particle filter for UWB localization.
/// </summary>
class SGD_UWB_ParticleFilter
{
	
protected:
	
	struct Location
	{
		double x;
		double y;
		double e;
	};

public:
	SGD_UWB_ParticleFilter();
	
	/// <summary>
	/// Call this to feed in a beacon position telegram.
	/// </summary>
	/// <param name="_r">Distance to beacon (meters)</param>
	/// <param name="id">Beacon ic.</param>
	void updateUwbMeasurement(double _r,
		SGD_UWB_BEACON_ID_TYPE id);

	/// <summary>
	/// Call this isochronously to feed in the odometry information and to update the filter solution.
	/// </summary>
	/// <param name="_dx">Position increment (x) from odometry class (meters)</param>
	/// <param name="_dy">Position increment (y) from odometry class (meters)</param>
	void updateOdometry(double _dx,
		double _dy);

	/// <summary>
	/// Resets the filter.
	/// </summary>
	void init();

	/// <summary>
	/// Asks the filter, if it could provide a localization solution. 
	/// </summary>
	/// <returns>True, if filter is in capture range.</returns>
	bool hasSolution();

	/// <summary>
	/// Calculates an estimate of the localization error.
	/// </summary>
	/// <returns>Position error (meters)</returns>
	double meanError();

	/// <summary>
	/// The localization solution x-coordinate.
	/// </summary>
	/// <returns>Position (meters)</returns>
	double x();

	/// <summary>
	/// The localization solution y-coordinate.
	/// </summary>
	/// <returns>Position (meters)</returns>
	double y();

protected:

	uint64_t cycle;

	Location est;
	Location pf;

	enum sState
	{
		Initializing,
		GatheringData,
		Locating
	} state;


	class Particle
	{
	public:
		double x;
		double y;
		double p;

		bool operator<(const Particle &rhs)
		{
			return p > rhs.p;
		}
	};

	const static unsigned int MaxPopulationSizeHint = 1000;
	const static unsigned int MinPopulation = 500;
	const static unsigned int SelectedParticles = 100;
	static constexpr double Radius = 0.1;
	static constexpr double Gain = 0.01;



	std::vector<Particle> particles;
	uint64_t lastSelectionTimestamp;
	bool particlesUpdated;

	class Beacon
	{
	public:
		SGD_UWB_BEACON_ID_TYPE id;
		double x;
		double y;
		double r;

		double age_s(uint64_t _timestamp);
		bool isValid(uint64_t _timestamp);

		uint64_t timestamp;
		bool valid;	
	};

	std::vector<Beacon> beacons;

public:
	void registerBeacon(
		double _x,
		double _y,
		SGD_UWB_BEACON_ID_TYPE _id);

protected:
    Location fuseParticles();

	void evaluateParticles(const std::vector<SGD_UWB_ParticleFilter::Beacon>::iterator& beaconIter);


#ifdef MAA_DEVEL
	friend int main();
#endif
};

