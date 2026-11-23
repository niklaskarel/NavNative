#include "nav/Logger/CsvRecorder.h"

#include "nav/Geometry/NavigationTypes.h"

#include <iomanip>
#include <locale>

namespace nav::logger
{
	namespace
	{
		constexpr int CsvPrecision = 15;
	}

	// Opens the selected CSV and writes the header used by frame exports.
	bool CsvRecorder::start(const std::filesystem::path& path)
	{
		m_file.open(path, std::ios::binary | std::ios::trunc);
		if (!m_file)
			return false;

		m_file.imbue(std::locale::classic());
		m_startedAt = std::chrono::steady_clock::now();
		m_file << std::setprecision(CsvPrecision)
			   << "elapsed_ms,truth_x_m,truth_y_m,gps_x_m,gps_y_m,match_x_m,match_y_m,"
				  "true_progress_m,estimated_progress_m,distance_from_route_m,off_route\n";
		return static_cast<bool>(m_file.flush());
	}

	// Closes the recording file when the viewer stops CSV capture.
	void CsvRecorder::stop()
	{
		if (m_file.is_open())
			m_file.close();
	}

	// Writes one drive frame with its elapsed recording time to the CSV file.
	bool CsvRecorder::append(const nav::session::DriveFrame& frame)
	{
		const auto elapsed =
			std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_startedAt);
		const auto& match = frame.match;
		m_file << elapsed.count() << ',' << frame.truth.x << ',' << frame.truth.y << ',' << frame.gps.x << ',' << frame.gps.y
			   << ',' << match.position.x << ',' << match.position.y << ',' << frame.progressMeters << ','
			   << match.progressMeters << ',' << match.distanceFromRouteMeters << ',' << (match.offRoute ? 1 : 0) << '\n';
		return static_cast<bool>(m_file.flush());
	}

	// Tells the viewer whether a CSV file is open.
	bool CsvRecorder::isRecording() const
	{
		return m_file.is_open();
	}
}    // namespace nav::logger
