#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <optional>

namespace nav::geometry
{
	/// Computes the straight-line distance in meters between two local points.
	double distance(Point a, Point b);

	/// Projects onto a finite segment in local meters. A zero-length segment yields no result.
	std::optional<SegmentProjection> projectOntoSegment(Point observation, Point start, Point end);
}    // namespace nav::geometry
