#pragma once

#include "nav/RouteTracker.h"

#include <vector>

/// Dimensions in local meters and route coordinates for the viewer's street grid.
namespace city
{
	inline constexpr int BlockPitch = 104;
	inline constexpr int Columns = 4;
	inline constexpr int Rows = 3;
	inline constexpr int MapWidth = Columns * BlockPitch;
	inline constexpr int MapHeight = Rows * BlockPitch;

	inline const std::vector<nav::Point>& route()
	{
		// Positions follow the right-hand lane through the street grid.
		static const std::vector<nav::Point> points{
			{0, 102},
			{210, 102},
			{210, 210},
			{106, 210},
			{106, 310},
			{MapWidth, 310},
		};
		return points;
	}
}    // namespace city
