#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <chrono>
#include <optional>

namespace nav::tracking
{
	/// Matches readings against a route, optionally using progress from earlier fixes.
	class RouteTracker
	{
	public:
		/// Stores the route and prepares the cumulative distances used for matching and interpolation.
		explicit RouteTracker(geometry::Route route);
		/// Stores the route and prepares the cumulative distances used for matching and interpolation.
		explicit RouteTracker(geometry::SharedRoute route);
		/// Matches a single observation against the full route without using fix history.
		std::optional<Match> update(geometry::Point observation) const;
		/// Matches a timestamped fix using a progress window derived from the previous accepted match.
		std::optional<Match> track(const GpsFix& fix);
		/// Clears timing history so the next fix can search the full route.
		void resetTracking();
		/// Returns the cached route length used to clamp progress and compute remaining distance.
		double lengthMeters() const;

		/// Interpolates a position at the given distance along the route.
		geometry::Point pointAt(double progressMeters) const;

	private:
		/// Finds the nearest route position inside the allowed progress window.
		std::optional<Match> matchWithin(geometry::Point observation, double minProgress, double maxProgress) const;

		geometry::SharedRoute m_route;       // Shared immutable route used by matching and interpolation.
		std::vector<double> m_cumulative;    // Distance from route start at each vertex for matching and interpolation.
		std::optional<std::chrono::steady_clock::time_point>
			m_lastFixTime;    // Timestamp of the previous valid fix used to reject older observations.
		std::optional<std::chrono::steady_clock::time_point>
			m_lastMatchTime;          // Timestamp of the last on-route match for the continuity window.
		double m_lastProgress = 0;    // Along-route position of the last accepted match.
	};
}    // namespace nav::tracking
