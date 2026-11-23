#include "nav/Simulation/DriveSimulation.h"

#include <algorithm>
#include <utility>

namespace nav::simulation
{
	namespace
	{
		constexpr std::mt19937::result_type NoiseSeed = 42;
	}

	// Prepares the route geometry used to advance and sample the simulated vehicle.
	DriveSimulation::DriveSimulation(nav::geometry::SharedRoute route)
		: m_routeGeometry(std::move(route))
		, m_random(NoiseSeed)
	{
	}

	// Moves the vehicle along the route and reports whether more distance remains.
	bool DriveSimulation::advance(double distanceMeters)
	{
		m_progressMeters = std::min(m_progressMeters + distanceMeters, m_routeGeometry.lengthMeters());
		return m_progressMeters < m_routeGeometry.lengthMeters();
	}

	// Returns the vehicle to the route start and reseeds the deterministic noise generator.
	void DriveSimulation::reset()
	{
		m_progressMeters = 0;
		m_random.seed(NoiseSeed);
	}

	// Returns ground truth and a simulated GPS position with configurable noise and offset.
	DriveSample DriveSimulation::sample(double noiseMeters, double northOffsetMeters)
	{
		const auto truth = m_routeGeometry.pointAt(m_progressMeters);
		std::normal_distribution<double> noise(0, 1);
		const nav::geometry::Point gps{truth.x + noiseMeters * noise(m_random),
									   truth.y + noiseMeters * noise(m_random) + northOffsetMeters};
		return {
			.progressMeters = m_progressMeters,
			.truth = truth,
			.gps = gps,
		};
	}
}    // namespace nav::simulation
