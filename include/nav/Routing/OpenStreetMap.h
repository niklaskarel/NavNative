#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <iosfwd>
#include <optional>

namespace nav::routing
{
	/// Imports drivable ways from an OpenStreetMap XML extract.
	std::optional<StreetMap> loadStreetMapFromOsmXml(std::istream& input);
}    // namespace nav::routing
