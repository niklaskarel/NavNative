#include "DriveSimulation.h"

#include "CityMap.h"

DriveSimulation::DriveSimulation()
	: m_tracker(city::route())
{
}

bool DriveSimulation::advance(double distanceMeters)
{
	if (m_progressMeters + distanceMeters > m_tracker.lengthMeters())
	{
		return false;
	}
	m_progressMeters += distanceMeters;
	return true;
}

void DriveSimulation::reset()
{
	m_progressMeters = 0;
	m_random.seed(42);
}

DriveFrame DriveSimulation::sample(double noiseMeters, double northOffsetMeters)
{
	const auto truth = nav::pointAt(m_tracker, m_progressMeters);
	std::normal_distribution<double> noise(0, 1);
	const nav::Point gps{truth.x + noiseMeters * noise(m_random), truth.y + noiseMeters * noise(m_random) + northOffsetMeters};
	return {
		.progressMeters = m_progressMeters,
		.truth = truth,
		.gps = gps,
		.match = m_tracker.update(gps),
	};
}
