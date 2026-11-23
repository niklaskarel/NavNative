#include "nav/Routing/LocalProjection.h"

#include <cmath>
#include <numbers>

namespace nav::routing
{
	namespace
	{
		constexpr double DegreesPerHalfTurn = 180.0;
		constexpr double DegreesPerTurn = 2 * DegreesPerHalfTurn;
		constexpr double EarthRadiusMeters = 6371000.0;

		constexpr double MetersPerDegree = EarthRadiusMeters * std::numbers::pi / DegreesPerHalfTurn;
	}

	// Stores the geographic origin and latitude-dependent longitude scale.
	LocalProjection::LocalProjection(nav::geometry::Point origin)
		: m_origin(origin)
		, m_longitudeScale(MetersPerDegree * std::cos(origin.y * std::numbers::pi / DegreesPerHalfTurn))
	{
	}

	// Converts longitude and latitude to local meters for routing and tracking.
	nav::geometry::Point LocalProjection::project(nav::geometry::Point degrees) const
	{
		return {std::remainder(degrees.x - m_origin.x, DegreesPerTurn) * m_longitudeScale,
				(degrees.y - m_origin.y) * MetersPerDegree};
	}

	// Converts local meters to longitude and latitude for simulated receiver output.
	nav::geometry::Point LocalProjection::unproject(nav::geometry::Point meters) const
	{
		return {std::remainder(m_origin.x + meters.x / m_longitudeScale, DegreesPerTurn),
				m_origin.y + meters.y / MetersPerDegree};
	}
}    // namespace nav::routing
