#include "nav/Logger/CsvRecorder.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <locale>
#include <mutex>
#include <thread>
#include <utility>

namespace nav::logger
{
	namespace
	{
		constexpr int CsvPrecision = 15;
		constexpr std::size_t MaxPendingRows = 256;
	}

	CsvRecorder::~CsvRecorder()
	{
		stop();
		join();
	}

	bool CsvRecorder::start(std::filesystem::path path, Completion completed)
	{
		if (m_thread.joinable() || !completed)
			return false;

		std::lock_guard lock(m_mutex);
		m_startedAt = std::chrono::steady_clock::now();
		m_accepting = true;
		m_failed = false;
		try
		{
			m_thread = std::jthread([this, path = std::move(path), completed = std::move(completed)] {
				bool success = false;
				try
				{
					success = writeFile(path);
				}
				catch (...)
				{
					// Report file and stream failures through the completion callback.
				}

				{
					std::lock_guard lock(m_mutex);
					m_accepting = false;
					success = success && !m_failed;
					m_rows.clear();
				}
				try
				{
					completed(success);
				}
				catch (...)
				{
					// A notification failure must not escape the thread.
				}
			});
		}
		catch (...)
		{
			m_accepting = false;
			return false;
		}
		return true;
	}

	void CsvRecorder::append(const nav::session::DriveFrame& frame)
	{
		{
			std::lock_guard lock(m_mutex);
			if (!m_accepting)
				return;

			try
			{
				if (m_rows.size() < MaxPendingRows)
				{
					const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::steady_clock::now() - m_startedAt);
					m_rows.push_back({elapsed, frame});
				}
				else
					m_failed = true;
			}
			catch (...)
			{
				m_failed = true;
			}

			if (m_failed)
				m_accepting = false;
		}
		m_changed.notify_one();
	}

	void CsvRecorder::stop()
	{
		{
			std::lock_guard lock(m_mutex);
			m_accepting = false;
		}
		m_changed.notify_one();
	}

	void CsvRecorder::join()
	{
		if (m_thread.joinable())
			m_thread.join();
	}

	bool CsvRecorder::writeFile(const std::filesystem::path& path)
	{
		std::ofstream file(path, std::ios::binary | std::ios::trunc);
		if (!file)
			return false;

		file.imbue(std::locale::classic());
		file << std::setprecision(CsvPrecision)
			 << "elapsed_ms,truth_x_m,truth_y_m,gps_x_m,gps_y_m,match_x_m,match_y_m,"
				"true_progress_m,estimated_progress_m,distance_from_route_m,off_route\n";

		while (file)
		{
			std::unique_lock lock(m_mutex);
			m_changed.wait(lock, [this] { return !m_rows.empty() || !m_accepting; });
			if (m_rows.empty())
				break;

			const auto row = std::move(m_rows.front());
			m_rows.pop_front();
			lock.unlock();

			const auto& frame = row.frame;
			const auto& match = frame.match;
			file << row.elapsed.count() << ',' << frame.truth.x << ',' << frame.truth.y << ','
				 << frame.gps.x << ',' << frame.gps.y << ',' << match.position.x << ',' << match.position.y
				 << ',' << frame.progressMeters << ',' << match.progressMeters << ',' << match.distanceFromRouteMeters
				 << ',' << (match.offRoute ? 1 : 0) << '\n';
		}

		file.flush();
		file.close();
		return static_cast<bool>(file);
	}
}    // namespace nav::logger
