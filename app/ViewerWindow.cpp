#include "ViewerWindow.h"

#include "MapCanvas.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

ViewerWindow::ViewerWindow()
	: m_canvas(new MapCanvas)
	, m_noise(new QSlider(Qt::Horizontal))
	, m_speed(new QSlider(Qt::Horizontal))
	, m_offset(new QCheckBox("Inject 60 m GPS offset"))
	, m_status(new QLabel)
	, m_recordButton(new QPushButton("Record CSV"))
{
	setWindowTitle("Navigation Lab | nearest-segment baseline");
	resize(1000, 760);

	auto* pause = new QPushButton("Pause");
	auto* resetButton = new QPushButton("Reset");
	auto* noiseLabel = new QLabel("GPS noise: 8 m");
	auto* speedLabel = new QLabel("Speed: 12 m/s");
	m_noise->setRange(0, 30);
	m_noise->setValue(8);
	m_noise->setAccessibleName("GPS noise in meters");
	m_speed->setRange(0, 30);
	m_speed->setValue(12);
	m_speed->setAccessibleName("Vehicle speed in meters per second");

	auto* controls = new QHBoxLayout;
	controls->addWidget(pause);
	controls->addWidget(resetButton);
	controls->addWidget(m_recordButton);
	controls->addWidget(m_offset);
	controls->addWidget(noiseLabel);
	controls->addWidget(m_noise);
	controls->addWidget(speedLabel);
	controls->addWidget(m_speed);
	auto* layout = new QVBoxLayout(this);
	layout->addLayout(controls);
	layout->addWidget(m_canvas);
	layout->addWidget(m_status);

	connect(pause, &QPushButton::clicked, this, [this, pause] {
		if (m_timer.isActive())
		{
			m_timer.stop();
			pause->setText("Resume");
		}
		else
		{
			m_timer.start();
			pause->setText("Pause");
		}
	});
	connect(resetButton, &QPushButton::clicked, this, &ViewerWindow::reset);
	connect(m_recordButton, &QPushButton::clicked, this, &ViewerWindow::toggleRecording);
	connect(m_noise, &QSlider::valueChanged, noiseLabel, [noiseLabel](int noise) {
		noiseLabel->setText(QString("GPS noise: %1 m").arg(noise));
	});
	connect(m_speed, &QSlider::valueChanged, speedLabel, [speedLabel](int speed) {
		speedLabel->setText(QString("Speed: %1 m/s").arg(speed));
	});
	connect(&m_timer, &QTimer::timeout, this, &ViewerWindow::advance);
	reset();
	m_timer.start(100);
}

void ViewerWindow::advance()
{
	const double distanceMeters = m_speed->value() * (m_timer.interval() / 1000.0);
	if (!m_simulation.advance(distanceMeters))
	{
		reset();
		return;
	}
	displayFrame();
}

void ViewerWindow::reset()
{
	m_simulation.reset();
	m_canvas->clearObservations();
	displayFrame();
}

void ViewerWindow::displayFrame()
{
	const double offset = m_offset->isChecked() ? 60 : 0;
	const auto frame = m_simulation.sample(m_noise->value(), offset);
	const auto& match = frame.match;
	m_canvas->setFrame(frame.truth, frame.gps, match);
	if (m_recorder.isRecording() && !m_recorder.append(frame))
	{
		recordingFailed();
	}
	m_status->setText(QString("True progress: %1 m | Estimated: %2 m | Remaining: %3 m | "
							  "Distance to route: %4 m | %5")
						  .arg(frame.progressMeters, 0, 'f', 1)
						  .arg(match.progressMeters, 0, 'f', 1)
						  .arg(match.remainingMeters, 0, 'f', 1)
						  .arg(match.distanceFromRouteMeters, 0, 'f', 1)
						  .arg(match.offRoute ? "OFF ROUTE" : "ON ROUTE"));
}

void ViewerWindow::toggleRecording()
{
	if (m_recorder.isRecording())
	{
		m_recorder.stop();
		m_recordButton->setText("Record CSV");
		return;
	}

	QFileDialog dialog(this, "Record GPS readings");
	dialog.setAcceptMode(QFileDialog::AcceptSave);
	dialog.setNameFilter("CSV files (*.csv)");
	dialog.setDefaultSuffix("csv");
	dialog.selectFile("RouteRecording.csv");
	if (dialog.exec() != QDialog::Accepted)
	{
		return;
	}
	if (!m_recorder.start(dialog.selectedFiles().constFirst()))
	{
		recordingFailed();
		return;
	}
	m_recordButton->setText("Stop Recording");
}

void ViewerWindow::recordingFailed()
{
	const auto message = m_recorder.errorString();
	m_recorder.stop();
	m_recordButton->setText("Record CSV");
	QMessageBox::warning(this, "CSV recording failed", message);
}
