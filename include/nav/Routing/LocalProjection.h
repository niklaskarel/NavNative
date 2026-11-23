#pragma once

#include "nav/Geometry/NavigationTypes.h"

namespace nav::routing
{
	/// Converts geographic coordinates (longitude, latitude) to local east/north meters.
	class LocalProjection
	{
	public:
		/// Stores the geographic origin and latitude-dependent longitude scale.
		explicit LocalProjection(nav::geometry::Point origin);
		/// Converts longitude and latitude to local meters for routing and tracking.
		nav::geometry::Point project(nav::geometry::Point degrees) const;
		/// Converts local meters to longitude and latitude for simulated receiver output.
		nav::geometry::Point unproject(nav::geometry::Point meters) const;

	private:
		nav::geometry::Point m_origin;    // Geographic origin of the local map coordinate system.
		double m_longitudeScale;          // Meters per longitude degree at the map origin.
	};
}    // namespace nav::routing
