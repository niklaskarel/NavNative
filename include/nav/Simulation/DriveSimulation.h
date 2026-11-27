#pragma once

#include "nav/Geometry/NavigationTypes.h"
#include "nav/Tracking/RouteTracker.h"

#include <random>

namespace nav::simulation
{
	/// Advances a car along the city route and generates noisy GPS fixes.
	class DriveSimulation
	{
	public:
		/// Prepares the route geometry used to advance and sample the simulated vehicle.
		explicit DriveSimulation(nav::geometry::SharedRoute route);
		/// Advances along the route and returns false once the destination is reached.
		bool advance(double distanceMeters);
		/// Returns the distance left on the simulated route.
		double remainingMeters() const { return m_routeGeometry.lengthMeters() - m_progressMeters; }
		/// Returns the vehicle to the route start and reseeds the deterministic noise generator.
		void reset();
		/// Returns ground truth and a simulated GPS position with configurable noise.
		DriveSample sample(double noiseMeters);

	private:
		nav::tracking::RouteTracker m_routeGeometry;    // Provides route length and interpolation for simulated motion.
		std::mt19937 m_random;                          // Seeded generator for reproducible GPS noise.
		double m_progressMeters = 0;                    // Distance traveled by the simulated vehicle along its route.
	};
}    // namespace nav::simulation
