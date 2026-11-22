#pragma once

#include <QElapsedTimer>
#include <QFile>
#include <QString>

struct DriveFrame;

/// Writes viewer frames to a CSV file while recording is active.
class CsvRecorder
{
public:
	bool start(const QString& path);
	void stop();
	bool append(const DriveFrame& frame);
	bool isRecording() const;
	QString errorString() const;

private:
	bool write(const QByteArray& bytes);

	QFile m_file;
	QElapsedTimer m_clock;
};
