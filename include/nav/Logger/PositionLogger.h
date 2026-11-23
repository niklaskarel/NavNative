#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <chrono>
#include <optional>

namespace nav::logger
{
	class Log;

	/// Submits every true position and limits the viewer to one entry every three seconds.
	class PositionLogger
	{
	public:
		/// Connects periodic position messages to the application log.
		explicit PositionLogger(Log& output);
		/// Clears the log interval so the first frame of the next drive is logged.
		void reset();
		/// Queues the true position and selects whether it appears in the viewer.
		void log(geometry::Point position, std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

	private:
		Log& m_output;    // Receives periodic position messages for asynchronous delivery.
		std::optional<std::chrono::steady_clock::time_point>
			m_lastLog;    // Time of the previous position message for enforcing the logging interval.
	};
}    // namespace nav::logger
