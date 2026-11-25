#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <thread>

namespace nav::logger
{
	/// Sends drive frames to a background CSV writer.
	class CsvRecorder
	{
	public:
		/// Called on the writer thread after the file has been closed.
		using Completion = std::function<void(bool success)>;

		~CsvRecorder();

		/// Requests a recording and transfers ownership of the path and callback.
		bool start(std::filesystem::path path, Completion completed);
		/// Queues a copy of a frame while the recorder accepts frames.
		void append(const nav::session::DriveFrame& frame);
		/// Stops accepting frames and lets the writer drain the queue.
		void stop();
		/// Joins the writer after completion or during destruction.
		void join();

	private:
		struct Row
		{
			std::chrono::milliseconds elapsed;
			nav::session::DriveFrame frame;
		};

		bool writeFile(const std::filesystem::path& path);

		std::mutex m_mutex;
		std::condition_variable m_changed;
		std::deque<Row> m_rows;
		std::chrono::steady_clock::time_point m_startedAt;
		bool m_accepting = false;
		bool m_failed = false;
		std::jthread m_thread;
	};
}    // namespace nav::logger
