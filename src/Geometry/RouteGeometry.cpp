#include "nav/Geometry/RouteGeometry.h"

#include <algorithm>
#include <cmath>

namespace nav::geometry
{
	// Computes the straight-line distance in meters between two local points.
	double distance(Point a, Point b)
	{
		return std::hypot(a.x - b.x, a.y - b.y);
	}

	// Finds the nearest point and distance along a finite segment for matching or hit testing.
	std::optional<SegmentProjection> projectOntoSegment(Point observation, Point start, Point end)
	{
		const double dx = end.x - start.x;
		const double dy = end.y - start.y;
		const double lengthSquared = dx * dx + dy * dy;
		if (lengthSquared == 0)
		{
			return std::nullopt;
		}
		// Divide the dot product by the squared segment length, then clamp the fraction to the segment.
		const double fraction =
			std::clamp(((observation.x - start.x) * dx + (observation.y - start.y) * dy) / lengthSquared, 0.0, 1.0);
		return SegmentProjection{
			.position = {std::lerp(start.x, end.x, fraction), std::lerp(start.y, end.y, fraction)},
			.distanceFromStart = fraction * std::sqrt(lengthSquared),
		};
	}

}    // namespace nav::geometry
