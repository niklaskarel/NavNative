#pragma once

#include "nav/RouteTracker.h"

#include <optional>

namespace nav
{
	/// The closest point and its distance along a segment from the start.
	struct SegmentProjection
	{
		Point position;
		double distanceFromStart;
	};

	double distance(Point a, Point b);

	/// Projects onto a finite segment in local meters. A zero-length segment yields no result.
	std::optional<SegmentProjection> projectOntoSegment(Point observation, Point start, Point end);
}    // namespace nav
