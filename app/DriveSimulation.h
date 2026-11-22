#pragma once

#include "nav/RouteTracker.h"

#include <random>

/// One simulated GPS update and its corresponding route match.
struct DriveFrame
{
	double progressMeters;
	nav::Point truth;
	nav::Point gps;
	nav::Match match;
};

/// Advances a car along the city route and generates noisy GPS fixes.
class DriveSimulation
{
public:
	DriveSimulation();
	/// Returns false when the distance would take the car past the end of the route.
	bool advance(double distanceMeters);
	void reset();
	DriveFrame sample(double noiseMeters, double northOffsetMeters);

private:
	nav::RouteTracker m_tracker;
	std::mt19937 m_random{42};
	double m_progressMeters = 0;
};
