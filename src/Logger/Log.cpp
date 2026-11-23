#include "nav/Logger/Log.h"

#include <iostream>
#include <sstream>
#include <utility>

namespace nav::logger
{
	namespace
	{
		constexpr std::size_t QueueCapacity = 512;

		// Maps each severity to its label in the console and viewer.
		std::string_view prefix(Level level)
		{
			switch (level)
			{
			case Level::Info:
				return "[INFO] ";
			case Level::Warning:
				return "[WARN] ";
			case Level::Error:
				return "[ERROR] ";
			}
			return {};
		}

		Log::DisplayEntry formatDisplayEntry(Level level, std::string_view line)
		{
			std::string escaped;
			escaped.reserve(line.size());
			for (const char character : line)
			{
				switch (character)
				{
				case '&': escaped += "&amp;"; break;
				case '<': escaped += "&lt;"; break;
				case '>': escaped += "&gt;"; break;
				case '"': escaped += "&quot;"; break;
				default: escaped += character; break;
				}
			}

			const auto html = [&escaped](const char* color) {
				return std::string("<span style=\"color:") + color + "\">" + escaped + "</span>";
			};
			switch (level)
			{
			case Level::Warning: return {html("#9a3412"), html("#fdba74")};
			case Level::Error: return {html("#b91c1c"), html("#fca5a5")};
			default: return {html("#166534"), html("#86efac")};
			}
		}
	}

	// Starts the thread that writes queued messages to the supplied stream.
	Log::Log(std::ostream& output)
		: m_output(output)
		, m_thread([this](std::stop_token stop) { writeMessages(stop); })
	{
	}

	// Copies a message into the bounded queue and wakes the logging thread.
	void Log::enqueue(Level level, std::string_view message) noexcept
	{
		try
		{
			push({level, std::string(message), true});
		}
		catch (...)
		{
			// Logging is best effort even when invoked from a failure handler.
		}
	}

	void Log::enqueuePosition(geometry::Point position, bool showInViewer) noexcept
	{
		try
		{
			push({Level::Info, position, showInViewer});
		}
		catch (...)
		{
			// Logging must not disrupt tracking.
		}
	}

	void Log::push(Entry entry)
	{
		{
			std::lock_guard lock(m_mutex);
			if (m_pending.size() == QueueCapacity)
				m_pending.pop_front();
			m_pending.push_back(std::move(entry));
		}
		m_changed.notify_one();
	}

	void Log::setDisplayHandler(DisplayHandler handler)
	{
		std::lock_guard lock(m_displayMutex);
		if (handler)
		{
			for (const auto& entry : m_display)
				handler(entry);
			m_display.clear();
		}
		m_displayHandler = std::move(handler);
	}

	// Waits for log entries, formats them, and publishes them to the output sinks.
	void Log::writeMessages(std::stop_token stop) noexcept
	{
		try
		{
			for (;;)
			{
				std::unique_lock lock(m_mutex);
				m_changed.wait(lock, stop, [this] { return !m_pending.empty(); });
				if (m_pending.empty())
					return;
				auto [level, payload, showInViewer] = std::move(m_pending.front());
				m_pending.pop_front();
				lock.unlock();

				std::string line(prefix(level));
				if (const auto* position = std::get_if<geometry::Point>(&payload))
				{
					std::ostringstream message;
					message << "True position: (" << position->x << ", " << position->y << ") m";
					line += message.str();
				}
				else
					line += std::get<std::string>(payload);
				m_output << line << std::endl;

				if (!showInViewer)
					continue;
				auto display = formatDisplayEntry(level, line);
				std::lock_guard displayLock(m_displayMutex);
				if (m_displayHandler)
					m_displayHandler(display);
				else
				{
					if (m_display.size() == QueueCapacity)
						m_display.pop_front();
					m_display.push_back(std::move(display));
				}
			}
		}
		catch (...)
		{
			// A failed diagnostic sink must not terminate the application.
		}
	}

	// Returns the shared logger used by the application and its modules.
	Log& applicationLog()
	{
		static Log log(std::clog);
		return log;
	}

	// Queues a severity-tagged message through the application logger.
	static void submit(Level level, std::string_view message) noexcept
	{
		try
		{
			applicationLog().enqueue(level, message);
		}
		catch (...)
		{
			// Starting the logger may fail if the system cannot create a thread.
		}
	}

	// Queues an informational message for the logging thread.
	void info(std::string_view message) noexcept
	{
		submit(Level::Info, message);
	}

	// Queues a warning when an operation can continue after a problem.
	void warning(std::string_view message) noexcept
	{
		submit(Level::Warning, message);
	}

	// Queues an error message for the console and application log.
	void error(std::string_view message) noexcept
	{
		submit(Level::Error, message);
	}
}
