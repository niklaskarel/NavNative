#include "nav/RouteTracker.h"

#include "nav/RouteGeometry.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace nav
{
	RouteTracker::RouteTracker(std::vector<Point> route)
		: m_route(std::move(route))
	{
		if (m_route.size() < 2)
			throw std::invalid_argument("A route needs at least two points");
		m_cumulative.push_back(0);
		for (std::size_t i = 0; i < m_route.size(); ++i)
		{
			if (!std::isfinite(m_route[i].x) || !std::isfinite(m_route[i].y))
				throw std::invalid_argument("Route coordinates must be finite");
			if (i)
				m_cumulative.push_back(m_cumulative.back() + distance(m_route[i - 1], m_route[i]));
		}
		if (lengthMeters() == 0)
			throw std::invalid_argument("Route must have nonzero length");
	}

	double RouteTracker::lengthMeters() const
	{
		return m_cumulative.back();
	}

	Match RouteTracker::update(Point observation) const
	{
		if (!std::isfinite(observation.x) || !std::isfinite(observation.y))
			throw std::invalid_argument("Observation must be finite");
		Match best;
		best.distanceFromRouteMeters = std::numeric_limits<double>::infinity();
		for (std::size_t i = 0; i + 1 < m_route.size(); ++i)
		{
			const auto projection = projectOntoSegment(observation, m_route[i], m_route[i + 1]);
			if (!projection)
			{
				continue;
			}
			const double error = distance(observation, projection->position);
			if (error < best.distanceFromRouteMeters)
			{
				best.position = projection->position;
				best.distanceFromRouteMeters = error;
				// Progress includes every segment before the current match.
				best.progressMeters = m_cumulative[i] + projection->distanceFromStart;
				best.segment = static_cast<int>(i);
			}
		}
		best.remainingMeters = lengthMeters() - best.progressMeters;
		// TODO: Name the off-route threshold and test its boundary behavior.
		best.offRoute = best.distanceFromRouteMeters > 25;
		return best;
	}
}    // namespace nav
