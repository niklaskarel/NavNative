#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>

namespace nav::session
{
	struct DriveFrame;
}

namespace nav::logger
{
	/// Writes viewer frames to a CSV file while recording is active.
	class CsvRecorder
	{
	public:
		/// Opens the selected CSV and writes the header used by frame exports.
		bool start(const std::filesystem::path& path);
		/// Closes the recording file when the viewer stops CSV capture.
		void stop();
		/// Writes one drive frame with its elapsed recording time to the CSV file.
		bool append(const nav::session::DriveFrame& frame);
		/// Tells the viewer whether a CSV file is open.
		bool isRecording() const;

	private:
		std::ofstream m_file;                                 // Open stream for the current CSV recording.
		std::chrono::steady_clock::time_point m_startedAt;    // Monotonic time origin for the elapsed column in the CSV.
	};
}    // namespace nav::logger
