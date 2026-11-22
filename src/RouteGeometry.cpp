#include "nav/RouteGeometry.h"

#include <algorithm>
#include <cmath>

namespace nav
{
	double distance(Point a, Point b)
	{
		return std::hypot(a.x - b.x, a.y - b.y);
	}

	std::optional<SegmentProjection> projectOntoSegment(Point observation, Point start, Point end)
	{
		const double dx = end.x - start.x;
		const double dy = end.y - start.y;
		const double lengthSquared = dx * dx + dy * dy;
		if (lengthSquared == 0)
		{
			return std::nullopt;
		}
		// The dot product gives a fraction along the line. Clamp it to the actual segment.
		const double fraction =
			std::clamp(((observation.x - start.x) * dx + (observation.y - start.y) * dy) / lengthSquared, 0.0, 1.0);
		return SegmentProjection{
			.position = {std::lerp(start.x, end.x, fraction), std::lerp(start.y, end.y, fraction)},
			.distanceFromStart = fraction * std::sqrt(lengthSquared),
		};
	}

	Point pointAt(const RouteTracker& tracker, double progressMeters)
	{
		double remaining = std::clamp(progressMeters, 0.0, tracker.lengthMeters());
		const auto& route = tracker.route();
		// Consume whole segments until the remaining distance falls inside one.
		for (std::size_t i = 0; i + 1 < route.size(); ++i)
		{
			const double length = distance(route[i], route[i + 1]);
			if (length == 0)
				continue;
			if (remaining <= length)
			{
				const double t = remaining / length;
				return {std::lerp(route[i].x, route[i + 1].x, t), std::lerp(route[i].y, route[i + 1].y, t)};
			}
			remaining -= length;
		}
		return route.back();
	}
}    // namespace nav
