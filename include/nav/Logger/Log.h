#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <condition_variable>
#include <deque>
#include <functional>
#include <iosfwd>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <variant>

namespace nav::logger
{
	enum class Level
	{
		Info,
		Warning,
		Error
	};

	/// Queues messages from any thread and writes them on a dedicated logging thread.
	class Log
	{
	public:
		struct DisplayEntry
		{
			std::string lightHtml;
			std::string darkHtml;
		};
		using DisplayHandler = std::function<void(const DisplayEntry&)>;

		/// Starts the thread that writes queued messages to the supplied stream.
		explicit Log(std::ostream& output);
		/// Copies a message into the bounded queue and wakes the logging thread.
		void enqueue(Level level, std::string_view message) noexcept;
		/// Queues a raw position for terminal output and optional GUI delivery.
		void enqueuePosition(geometry::Point position, bool showInViewer) noexcept;
		/// Registers a GUI sink. Clearing it waits for any delivery already in progress.
		void setDisplayHandler(DisplayHandler handler);

	private:
		struct Entry
		{
			Level level;
			std::variant<std::string, geometry::Point> payload;
			bool showInViewer;
		};

		void push(Entry entry);
		/// Waits for log entries, formats them, and publishes them to the output sinks.
		void writeMessages(std::stop_token stop) noexcept;

		std::ostream& m_output;                   // Output stream written by the logging thread.
		std::mutex m_mutex;                       // Protects pending messages.
		std::condition_variable_any m_changed;    // Wakes the logging thread when an entry is queued.
		std::deque<Entry> m_pending;         // Pending entries awaiting formatting and output.
		std::mutex m_displayMutex;           // Serializes delivery and sink registration.
		std::deque<DisplayEntry> m_display;  // Prepared entries buffered until a viewer registers.
		DisplayHandler m_displayHandler;
		std::jthread m_thread;                // Drains queued entries onto the output stream and into the GUI buffer.
	};

	/// Returns the shared logger used by the application and its modules.
	Log& applicationLog();
	/// Queues an informational message for the logging thread.
	void info(std::string_view message) noexcept;
	/// Queues a warning when an operation can continue after a problem.
	void warning(std::string_view message) noexcept;
	/// Queues an error message for the console and application log.
	void error(std::string_view message) noexcept;
}
