#pragma once

#include "CsvRecorder.h"
#include "DriveSimulation.h"

#include <QTimer>
#include <QWidget>

class MapCanvas;
class QCheckBox;
class QLabel;
class QPushButton;
class QSlider;

/// Owns the Qt controls and clock and presents each simulated frame.
class ViewerWindow : public QWidget
{
public:
	ViewerWindow();

private:
	void advance();
	void reset();
	void displayFrame();
	void toggleRecording();
	void recordingFailed();

	DriveSimulation m_simulation;
	CsvRecorder m_recorder;
	QTimer m_timer;
	MapCanvas* m_canvas;
	QSlider* m_noise;
	QSlider* m_speed;
	QCheckBox* m_offset;
	QLabel* m_status;
	QPushButton* m_recordButton;
};
