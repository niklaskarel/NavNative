#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <optional>
#include <string>
#include <string_view>

namespace nav::gnss
{
	/// Encodes a position fix as a receiver sentence.
	std::string encodeRmc(const RmcFix& fix);

	/// Decodes a valid NMEA RMC position fix.
	std::optional<RmcFix> parseRmc(std::string_view sentence);
}    // namespace nav::gnss
