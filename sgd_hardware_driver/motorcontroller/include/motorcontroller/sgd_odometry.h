#pragma once
#include <cinttypes>
#define _USE_MATH_DEFINES
#include <cmath>
/// <summary>
/// An odometry class that improves the wheel encoder velocities by incorporating the compass sensor of the IMU and the GPS heading.
/// </summary>
class SGD_Odometry
{

public:

	SGD_Odometry();

	/// The output values. dx and dy are required by the UWB localization class.
	double dx, dy, dphi;

	///  The state variables for debugging and compatibility reasons.
	double x, y, phi;

	/// <summary>
	/// Call this to feed in the compass heading.
	/// </summary>
	/// <param name="_phi_IMU_rad">The compass heading in radians</param>
	void updatePhi_IMU(double _phi_IMU_rad);

	/// <summary>
	/// Call this to feed in the GPS position.
	/// </summary>
	/// <param name="_x_m">x-coordinate (meters) w.r.t. the map</param>
	/// <param name="_y_m">y-coordinate (meters) w.r.t. the map</param>
	void updateLoc_GPS(double _x_m, double _y_m);

	/// <summary>
	/// Initializes the state variables (optional). The state values default to zero.
	/// </summary>
	/// <param name="_x">x-cordinate (meters) of the SGD w.r.t. the map</param>
	/// <param name="_y">y-cordinate (meters) of the SGD w.r.t. the map</param>
	/// <param name="_phi">orientation (radians) of the SGD w.r.t. the east direction</param>
	void init(double _x, double _y, double _phi);

	/// <summary>
	/// Call this to feed in the wheel odometry results.
	/// </summary>
	/// <param name="_dx">position increment in forward direction (meters / sample time)</param>
	/// <param name="_dphi">angular increment (radians / sample time)</param>
	void update(double _dx, double _dphi);

	/// <summary>
	/// Sets the gain of the heading correction by the compass heading. Defaults to 0.045.
	/// </summary>
	/// <param name="_g">the gain (1 / sample time)</param>
	void setCompassGain(double _g);

	/// <summary>
	/// Sets the gain of the heading correction by the GPS heading. Defaults to 0.5.
	/// </summary>
	/// <param name="_g">the gain (1 / sample time)</param>
	void setGpsOrientationGain(double _g);

protected:
	uint64_t cycle;

	double g1;
	double g2;

	class sIMU
	{
	public:
		double phi;
		void reset();
		void insertSample(double _phi, uint64_t _timestamp);
		bool isValid(uint64_t _timestamp);
		double age_s(uint64_t _timestamp);

	protected:	
		uint64_t lastTimestamp;
		bool samplesValid;

	} imu;

	class sGPS
	{
	protected:
		static const int depth = 10;

		struct sSample
		{
			double x;
			double y;
			uint64_t timestamp;
			bool valid;
		} samples[depth+1];
	
		double sPhi[depth];
		bool samplesValid();

		int actIndex;
	public:
		bool isValid(uint64_t _timestamp);
		void reset();
		void insertSample(double _x, double _y, uint64_t _timestamp);
		double age_s(uint64_t _timestamp);
		double uncertainty();
		double phi();

	} gps;

public:
	// Corrects angle to -pi .. pi
	static double corrAngle(double in);

	// Sample time used for parameter tuning im MATLAB
	static constexpr double T_s_ref = 0.01;
};




