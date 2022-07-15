#ifdef MAA_DEVEL
#include "sgd_odometry.h"
#else
#include "motorcontroller/sgd_odometry.h"
#endif // MAA_DEVEL

void SGD_Odometry::init(double _x, double _y, double _phi)
{
	x = _x;
	y = _y;
	phi = _phi;

	cycle = 0;

	imu.reset();
	gps.reset();
}

SGD_Odometry::SGD_Odometry()
{
	init(0, 0, 0);
	setCompassGain(0.045);
	setGpsOrientationGain(0.5);
}

void SGD_Odometry::updatePhi_IMU(double _phi_IMU_rad)
{
	imu.insertSample(_phi_IMU_rad, cycle);
}

void SGD_Odometry::updateLoc_GPS(double _x_m, double _y_m)
{
	gps.insertSample(_x_m, _y_m, cycle);
}

void SGD_Odometry::update(double _dx_m, double _dphi_rad)
{
	double dt = T_s_ref;
	
	// A priori phi calculation
	phi += _dphi_rad * dt;
	phi = corrAngle(phi);
	double phi_ap = phi;
	dphi = _dphi_rad;

	// Correct phi with the IMU compass sensor
	if (imu.isValid(cycle))
	{
		double e = corrAngle(imu.phi - phi_ap);
		double k = g1 * dt / T_s_ref;

		phi += k * e;
		phi = corrAngle(phi);
		dphi += k * e / dt;
	}

	// Correct phi with the GPS orientation
	if (gps.isValid(cycle))
	{
		double k = g2 * exp(-gps.uncertainty() * 100.0 * 0.1) / 5.0 * dt / T_s_ref;
		double e;
		if (_dx_m >= 0)
		{
			e = corrAngle(gps.phi() - phi_ap);
		}
		else
		{
			e = corrAngle(corrAngle(gps.phi() + M_PI) - phi_ap);
		}
	
		phi += k * e;
		phi = corrAngle(phi);
		dphi += k * e / dt;
	}

	// Calculate position increments (the odometry information for the particle filter)
	dx = _dx_m * cos(phi);
	dy = _dx_m * sin(phi);

	// Integrate position in global coordinates
	x += dx * dt;
	y += dy * dt;

	cycle++;
}

void SGD_Odometry::setCompassGain(double _g)
{
	g1 = _g;
}

void SGD_Odometry::setGpsOrientationGain(double _g)
{
	g2 = _g;
}

double SGD_Odometry::corrAngle(double in)
{
	double o = in;
	if (o > M_PI)
	{
		o = o - 2 * M_PI;
	}
	else if (o < -M_PI)
	{
		o = o + 2 * M_PI;
	}
	
	return o;
}

bool SGD_Odometry::sGPS::samplesValid()
{
	bool valid = true;
	for (int i = 1; i < depth + 1; i++)
	{
		valid &= samples[i].valid;
	}
	return valid;
}

bool SGD_Odometry::sGPS::isValid(uint64_t _timestamp)
{
	return samplesValid() && age_s(_timestamp < 2.0);
}

void SGD_Odometry::sGPS::reset()
{
	for (int i = 0; i < depth+1; i++)
	{
		samples[i].valid = false;
		samples[i].timestamp = 0;
		actIndex = 0;
	}
}

void SGD_Odometry::sGPS::insertSample(double _x, double _y, uint64_t _timestamp)
{
	samples[actIndex].x = _x;
	samples[actIndex].y = _y;
	samples[actIndex].timestamp = _timestamp;
	samples[actIndex].valid = true;
	
	actIndex = (++actIndex) % (depth + 1);
}

double SGD_Odometry::sGPS::age_s(uint64_t _timestamp)
{
	int idx = actIndex - 1;
	if (idx < 0) idx += (depth + 1);
	return (_timestamp - samples[idx].timestamp) * T_s_ref;
}

double SGD_Odometry::sGPS::uncertainty()
{
	if (samplesValid())
	{
		int sampleIndex = actIndex;

		double mean_dx = 0.0;
		double mean_dy = 0.0;
		double mean_r = 0.0;

		for (int i = 0; i < depth; i++)
		{
			int idx1, idx2;
			if ((idx1 = --sampleIndex) < 0) idx1 += (depth + 1);
			if ((idx2 = sampleIndex - 1) < 0) idx2 += (depth + 1);

			double r_x = samples[idx1].x - samples[idx2].x;
			double r_y = samples[idx1].y - samples[idx2].y;

			sPhi[i] = atan2(r_y, r_x);

			mean_dx += cos(sPhi[i]);
			mean_dy += sin(sPhi[i]);
			
			mean_r += sqrt(r_x * r_x + r_y * r_y);
		}

		mean_dx /= depth;
		mean_dy /= depth;
		mean_r /= depth;

		double varianceSum = 0.0;
		for (int i = 0; i < depth; i++)
		{
			varianceSum += (mean_dx - cos(sPhi[i])) * (mean_dx - cos(sPhi[i])) + (mean_dy - sin(sPhi[i])) * (mean_dy - sin(sPhi[i]));
		}

		return varianceSum / depth * exp (mean_r);
	}
	else
	{
		return NAN;
	}
}

double SGD_Odometry::sGPS::phi()
{
	if (samplesValid())
	{
		return sPhi[0];
	}
	else
	{
		return NAN;
	}
}

void SGD_Odometry::sIMU::reset()
{
	lastTimestamp = 0;
	samplesValid = false;
}

void SGD_Odometry::sIMU::insertSample(double _phi, uint64_t _timestamp)
{
	phi = _phi;
	lastTimestamp = _timestamp;
	samplesValid = true;
}

bool SGD_Odometry::sIMU::isValid(uint64_t _timestamp)
{
	return samplesValid && age_s(_timestamp) < 0.06;
}

double SGD_Odometry::sIMU::age_s(uint64_t _timestamp)
{
	if (samplesValid)
	{
		return (_timestamp - lastTimestamp) * T_s_ref;
	}
	else
	{
		return NAN;
	}
}

