#include "CsvRecorder.h"

#include "DriveSimulation.h"

#include <QTextStream>

bool CsvRecorder::start(const QString& path)
{
	m_file.setFileName(path);
	if (!m_file.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		return false;
	}
	m_clock.start();
	return write("elapsed_ms,truth_x_m,truth_y_m,gps_x_m,gps_y_m,match_x_m,match_y_m,"
				 "true_progress_m,estimated_progress_m,distance_from_route_m,off_route\n");
}

void CsvRecorder::stop()
{
	m_file.close();
}

bool CsvRecorder::append(const DriveFrame& frame)
{
	QByteArray row;
	QTextStream stream(&row);
	stream.setRealNumberPrecision(15);
	const auto& match = frame.match;
	stream << m_clock.elapsed() << ',' << frame.truth.x << ',' << frame.truth.y << ',' << frame.gps.x << ',' << frame.gps.y
		   << ',' << match.position.x << ',' << match.position.y << ',' << frame.progressMeters << ',' << match.progressMeters
		   << ',' << match.distanceFromRouteMeters << ',' << (match.offRoute ? 1 : 0) << '\n';
	stream.flush();
	return write(row);
}

bool CsvRecorder::isRecording() const
{
	return m_file.isOpen();
}

QString CsvRecorder::errorString() const
{
	return m_file.errorString();
}

bool CsvRecorder::write(const QByteArray& bytes)
{
	return m_file.write(bytes) == bytes.size() && m_file.flush();
}
