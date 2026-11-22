#pragma once
#include <vector>

namespace nav
{
	/// A point in a local Cartesian coordinate system measured in meters.
	struct Point
	{
		double x = 0;
		double y = 0;
	};

	/// The nearest route position and distances measured along and away from the route.
	struct Match
	{
		Point position;
		double progressMeters = 0;
		double remainingMeters = 0;
		double distanceFromRouteMeters = 0;
		int segment = 0;
		bool offRoute = false;
	};

	/// Matches each GPS observation independently against a fixed route polyline.
	class RouteTracker
	{
	public:
		explicit RouteTracker(std::vector<Point> route);
		Match update(Point observation) const;
		double lengthMeters() const;

		const std::vector<Point>& route() const
		{
			return m_route;
		}

	private:
		std::vector<Point> m_route;
		std::vector<double> m_cumulative;
	};

	/// Interpolates a point along the route, clamping progress to the route endpoints.
	Point pointAt(const RouteTracker& tracker, double progressMeters);
}    // namespace nav
