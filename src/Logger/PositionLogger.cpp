#include "nav/Logger/PositionLogger.h"

#include "nav/Logger/Log.h"

namespace nav::logger
{
	namespace
	{
		constexpr auto LogInterval = std::chrono::seconds(3);
	}

	// Connects periodic position messages to the application log.
	PositionLogger::PositionLogger(Log& output)
		: m_output(output)
	{
	}

	// Clears the log interval so the first frame of the next drive is logged.
	void PositionLogger::reset()
	{
		m_lastLog.reset();
	}

	// Queues every position and applies the display interval independently.
	void PositionLogger::log(geometry::Point position, std::chrono::steady_clock::time_point now)
	{
		const bool showInViewer = !m_lastLog || now - *m_lastLog >= LogInterval;
		if (showInViewer)
			m_lastLog = now;
		m_output.enqueuePosition(position, showInViewer);
	}
}    // namespace nav::logger
