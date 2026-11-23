#include "nav/Tracking/RouteTracker.h"

#include "nav/Logger/Log.h"
#include "nav/Geometry/RouteGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace nav::tracking
{
	namespace
	{
		constexpr double OffRouteDistanceMeters = 25;
		constexpr double MaxSpeedMetersPerSecond = 35;
		constexpr double ProgressToleranceMeters = 10;
		constexpr auto ReacquireAfter = std::chrono::seconds(2);
	}

	// Stores the route and prepares the cumulative distances used for matching and interpolation.
	RouteTracker::RouteTracker(geometry::Route route)
		: RouteTracker(std::make_shared<const geometry::Route>(std::move(route)))
	{
	}

	// Stores the route and prepares the cumulative distances used for matching and interpolation.
	RouteTracker::RouteTracker(geometry::SharedRoute route)
		: m_route(std::move(route))
	{
		if (!m_route || m_route->size() < 2)
		{
			nav::logger::error("A route needs at least two points");
			return;
		}
		const auto& points = *m_route;
		m_cumulative.reserve(points.size());
		m_cumulative.push_back(0);
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y))
			{
				nav::logger::error("Route coordinates must be finite");
				m_cumulative.clear();
				return;
			}
			if (i)
				m_cumulative.push_back(m_cumulative.back() + geometry::distance(points[i - 1], points[i]));
		}
		if (!std::isfinite(lengthMeters()) || lengthMeters() == 0)
		{
			nav::logger::error("Route length must be finite and greater than zero");
			m_cumulative.clear();
		}
	}

	// Returns the cached route length used to clamp progress and compute remaining distance.
	double RouteTracker::lengthMeters() const
	{
		return m_cumulative.empty() ? 0.0 : m_cumulative.back();
	}

	// Matches a single observation against the full route without using fix history.
	std::optional<Match> RouteTracker::update(geometry::Point observation) const
	{
		return matchWithin(observation, 0, lengthMeters());
	}

	// Matches a timestamped fix using a progress window derived from the previous accepted match.
	std::optional<Match> RouteTracker::track(const GpsFix& fix)
	{
		if (m_lastFixTime && fix.timestamp < *m_lastFixTime)
		{
			nav::logger::warning("GPS fix discarded because it arrived out of timestamp order");
			return std::nullopt;
		}

		double minProgress = 0;
		double maxProgress = lengthMeters();
		if (m_lastMatchTime && fix.timestamp - *m_lastMatchTime < ReacquireAfter)
		{
			const double elapsed = std::chrono::duration<double>(fix.timestamp - *m_lastMatchTime).count();
			const double travel = ProgressToleranceMeters + elapsed * MaxSpeedMetersPerSecond;
			minProgress = std::max(0.0, m_lastProgress - travel);
			maxProgress = std::min(lengthMeters(), m_lastProgress + travel);
		}
		const auto match = matchWithin(fix.position, minProgress, maxProgress);
		if (!match)
			return std::nullopt;
		m_lastFixTime = fix.timestamp;
		if (!match->offRoute)
		{
			m_lastMatchTime = fix.timestamp;
			m_lastProgress = match->progressMeters;
		}
		return match;
	}

	// Clears timing history so the next fix can search the full route.
	void RouteTracker::resetTracking()
	{
		m_lastFixTime.reset();
		m_lastMatchTime.reset();
	}

	// Finds the nearest route position inside the allowed progress window.
	std::optional<Match> RouteTracker::matchWithin(geometry::Point observation, double minProgress, double maxProgress) const
	{
		if (!std::isfinite(observation.x) || !std::isfinite(observation.y))
		{
			nav::logger::warning("GPS fix discarded: observation must be finite");
			return std::nullopt;
		}
		if (m_cumulative.empty())
			return std::nullopt;
		Match best;
		best.distanceFromRouteMeters = std::numeric_limits<double>::infinity();
		const auto& points = *m_route;
		const auto firstEnd = m_cumulative.begin() + 1;
		// Include the preceding segment when the window starts exactly at a shared vertex.
		const auto first = std::lower_bound(firstEnd, m_cumulative.end(), minProgress);
		const auto firstSegment = static_cast<std::size_t>(first - firstEnd);
		for (std::size_t i = firstSegment; i + 1 < points.size(); ++i)
		{
			const double start = m_cumulative[i];
			if (start > maxProgress)
				break;
			const double end = m_cumulative[i + 1];
			const auto projection = geometry::projectOntoSegment(observation, points[i], points[i + 1]);
			if (!projection.has_value())
				continue;
			// Clip the projection along this segment to the permitted progress window.
			const double projectedProgress = start + projection->distanceFromStart;
			const double progress = std::clamp(projectedProgress, minProgress, maxProgress);
			const double fraction = (progress - start) / (end - start);
			const geometry::Point position = progress == projectedProgress
												 ? projection->position
												 : geometry::Point{std::lerp(points[i].x, points[i + 1].x, fraction),
																   std::lerp(points[i].y, points[i + 1].y, fraction)};
			const double error = geometry::distance(observation, position);
			if (error < best.distanceFromRouteMeters)
			{
				best.position = position;
				best.distanceFromRouteMeters = error;
				// Progress includes every segment before the current match.
				best.progressMeters = progress;
				best.segment = static_cast<int>(i);
			}
		}
		best.remainingMeters = lengthMeters() - best.progressMeters;
		best.offRoute = best.distanceFromRouteMeters > OffRouteDistanceMeters;
		return best;
	}

	// Interpolates a position at the requested distance for the simulator.
	geometry::Point RouteTracker::pointAt(double progressMeters) const
	{
		if (m_cumulative.empty())
			return {};
		const auto& route = *m_route;
		if (progressMeters <= 0)
			return route.front();
		if (!(progressMeters < lengthMeters()))
			return route.back();

		const auto end = std::lower_bound(m_cumulative.begin(), m_cumulative.end(), progressMeters);
		const auto index = static_cast<std::size_t>(end - m_cumulative.begin());
		const double start = m_cumulative[index - 1];
		const double fraction = (progressMeters - start) / (*end - start);
		const auto& from = route[index - 1];
		const auto& to = route[index];
		return {std::lerp(from.x, to.x, fraction), std::lerp(from.y, to.y, fraction)};
	}
}    // namespace nav::tracking
