#ifdef MAA_DEVEL
#include "sgd_uwb_pf.h"
#else
#include "uwb/sgd_uwb_pf.h"
#endif // MAA_DEVEL

void SGD_UWB_ParticleFilter::updateOdometry(double _dx, double _dy)
{

	// Move all particles
	for (auto &particle : particles)
	{
		particle.x += _dx * SGD_Odometry::T_s_ref;
		particle.y += _dy * SGD_Odometry::T_s_ref;
	}

	switch (state)
	{
	case Initializing:
		break;

	case GatheringData:
		if (particlesUpdated)
		{
			particlesUpdated = false;

			Location testLocation = fuseParticles();
			est.x = testLocation.x;
			est.y = testLocation.y;
			if (est.e < 1.0)
			{
				state = Locating;
			}
		}
		break;

	case Locating:
		// If new measurements came in, re-estimate position. Else just apply odometry information
		if (particlesUpdated)
		{
			particlesUpdated = false;
			pf = fuseParticles();

			// Put it through a correction
			if (est.e < 3.0)
			{
				est.x += _dx * SGD_Odometry::T_s_ref + Gain * (pf.x - est.x);
				est.y += _dy * SGD_Odometry::T_s_ref + Gain * (pf.y - est.y);
			}
			else
			// Particles are lost, re-init filter
			{
				est.x += _dx * SGD_Odometry::T_s_ref;;
				est.y += _dy * SGD_Odometry::T_s_ref;; 
				init();
				break;
			}
		}
		else
		{
			est.x += _dx * SGD_Odometry::T_s_ref;;
			est.y += _dy * SGD_Odometry::T_s_ref;;
		}



		// Selection and mutation of the best ones
	
		if (((cycle - lastSelectionTimestamp) * SGD_Odometry::T_s_ref) > 0.2 && particles.size() > MinPopulation)
		{
			particles.erase(particles.begin() + MinPopulation, particles.end());
			lastSelectionTimestamp = cycle;
		}

		for (size_t p = 0; p < SelectedParticles && particles.size() <= MaxPopulationSizeHint; p++)
		{
			for (int n = 0; n < 5; n++)
			{
				Particle particle;
				double r = Radius * rand() / (double) RAND_MAX;
				double phi = rand() / (double) RAND_MAX * 2 * M_PI;
				particle.x = particles[p].x + r * cos(phi);
				particle.y = particles[p].y + r * sin(phi);
				particle.p = particles[p].p;
				particles.push_back(particle);
			}
		}
		break;
	}
	cycle++;
}

void SGD_UWB_ParticleFilter::init()
{
		particlesUpdated = false;
		particles.clear();
		particles.reserve(MaxPopulationSizeHint + MaxPopulationSizeHint / 10);
		state = Initializing;
		cycle = 0;
		lastSelectionTimestamp = 0;
		est.x = NAN;
		est.y = NAN;
		est.e = INFINITY;
}

bool SGD_UWB_ParticleFilter::hasSolution()
{
	return state == Locating;
}

SGD_UWB_ParticleFilter::SGD_UWB_ParticleFilter()
{
	init();
}

void SGD_UWB_ParticleFilter::updateUwbMeasurement(double _r, SGD_UWB_BEACON_ID_TYPE _id)
{
	auto beaconIter = std::find_if(beacons.begin(), beacons.end(),
		[_id](Beacon beacon) {return beacon.id == _id; });

	if (beaconIter != beacons.end())
	{
		beaconIter->r = _r;
		beaconIter->timestamp = cycle;
		beaconIter->valid = true;

		switch (state)
		{
		case Initializing:

			// Solution for the position must be on a circle around the beacon
			for (int i = 0; i < 120; i++)
			{
				Particle nP;
				nP.x = beaconIter->x + beaconIter->r * sin(i * 2 * M_PI / 120.0);
				nP.y = beaconIter->y + beaconIter->r * cos(i * 2 * M_PI / 120.0);
				nP.p = 1.0;
				particles.push_back(nP);
			}
			est.x = beaconIter->x;
			est.y = beaconIter->y;
			est.e = 2*beaconIter->r;
			state = GatheringData;
			break;

		case GatheringData:
		case Locating:

			evaluateParticles(beaconIter);
			particlesUpdated = true;
			break;
		}
	}
}

void SGD_UWB_ParticleFilter::evaluateParticles(const std::vector<SGD_UWB_ParticleFilter::Beacon>::iterator& beaconIter)
{
	// Evaluate and normalize particles
	double pivot = 0.0;

	double x_b = beaconIter->x;
	double y_b = beaconIter->y;
	double r_b = beaconIter->r;

	for (auto& particle : particles)
	{
		double x_p = particle.x;
		double y_p = particle.y;

		double r = sqrt((x_p - x_b) * (x_p - x_b) + (y_p - y_b) * (y_p - y_b));

		particle.p *= exp(-(r - r_b) * (r - r_b) / 10);

		if (particle.p > pivot)
		{
			pivot = particle.p;
		}
	}

	if (pivot > 0.0)
	{
		for (auto& particle : particles)
		{
			particle.p /= pivot;
		}
	}

	double r = sqrt((est.x - x_b) * (est.x - x_b) + (est.y - y_b) * (est.y - y_b));

	int validBeacons = 0;
	for (auto &beacon : beacons)
	{
		if (beacon.isValid(cycle))
			validBeacons++;
	}

	double filterInnov = 1.0 / validBeacons;

	est.e = (1 - filterInnov) * est.e + filterInnov * sqrt((r - r_b) * (r - r_b));

}


SGD_UWB_ParticleFilter::Location SGD_UWB_ParticleFilter::fuseParticles()
{
	struct Location loc;

	std::sort(particles.begin(), particles.end());

	loc.x = 0.0;
	loc.y = 0.0;
	double sum_w = 0.0;
	
	size_t i;

	for (i = 0; i < particles.size() && i < SelectedParticles; i++)
	{
		double x = particles[i].x;
		double y = particles[i].y;
		double p = particles[i].p;

		loc.x += x * p * p;
		loc.y += y * p * p;
		sum_w += p * p;
	}

	loc.x /= sum_w;
	loc.y /= sum_w;
		
	return loc;
}

double SGD_UWB_ParticleFilter::meanError()
{
	return est.e;
}

double SGD_UWB_ParticleFilter::x()
{
	return est.x;
}

double SGD_UWB_ParticleFilter::y()
{
	return est.y;
}

void SGD_UWB_ParticleFilter::registerBeacon(double _x, double _y, SGD_UWB_BEACON_ID_TYPE _id)
{
	auto beaconIter = std::find_if(beacons.begin(), beacons.end(),
		[_id](Beacon beacon) {return beacon.id == _id; });

	if (beaconIter == beacons.end())
	{
		Beacon newBeacon;
		newBeacon.x = _x;
		newBeacon.y = _y;
		newBeacon.id = _id;
		newBeacon.valid = false;

		beacons.push_back(newBeacon);
	}
	else
	{
		beaconIter->x = _x;
		beaconIter->y = _y;
	}
}

double SGD_UWB_ParticleFilter::Beacon::age_s(uint64_t _timestamp)
{
	return (_timestamp - timestamp) * SGD_Odometry::T_s_ref;
}

bool SGD_UWB_ParticleFilter::Beacon::isValid(uint64_t _timestamp)
{
	return valid && age_s(_timestamp) < 2.0;
}
