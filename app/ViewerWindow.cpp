#include "ViewerWindow.h"

#include "MapCanvas.h"
#include "nav/Logger/Log.h"
#include "ui_ViewerWindow.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QEvent>
#include <QFileDialog>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>

#include <utility>

namespace nav::viewer
{
	namespace
	{
		constexpr int DarkBackgroundThreshold = 128;
		constexpr int WorkerCheckMilliseconds = 100;
		constexpr int ProgressDecimalPlaces = 1;

		using State = nav::session::RouteSession::State;

		template <typename Payload> class ViewerEvent : public QEvent
		{
		public:
			// Registers and returns the Qt event type for the given payload.
			static QEvent::Type typeId()
			{
				static const auto type = static_cast<QEvent::Type>(QEvent::registerEventType());
				return type;
			}

			// Stores a worker result for delivery through Qt's event queue.
			explicit ViewerEvent(Payload payload)
				: QEvent(typeId())
				, data(std::move(payload))
			{
			}

			Payload data;    // Payload delivered to the viewer by the Qt event.
		};

		struct DisplayFrame
		{
			nav::session::DriveFrame frame;    // Matched drive frame carried by a GUI event.
			std::size_t driveId;               // ID used to reject events from a previous drive.
		};

		using FrameEvent = ViewerEvent<DisplayFrame>;
		using PlanEvent = ViewerEvent<nav::session::RouteSession::PlanResult>;
		enum class AfterControl
		{
			None,
			Reset
		};
		using ControlEvent = ViewerEvent<AfterControl>;

		// Converts a session state into the label shown in the viewer.
		QString stateName(State state)
		{
			switch (state)
			{
			case State::NotStarted:
				return "Not started";
			case State::SettingRoute:
				return "Setting route";
			case State::Calculating:
				return "Calculating";
			case State::ChoosingRoute:
				return "Choosing route";
			case State::Starting:
				return "Starting";
			case State::Running:
				return "Running";
			case State::Pausing:
				return "Pausing";
			case State::Paused:
				return "Paused";
			case State::Stopping:
				return "Stopping";
			case State::Stopped:
				return "Stopped";
			}
			return {};
		}
	}

	// Loads the Designer form and connects its controls to the session and map.
	ViewerWindow::ViewerWindow(nav::routing::StreetMap map)
		: m_session(map)
		, m_ui(std::make_unique<Ui::ViewerWindow>())
		, m_canvas(new MapCanvas(std::move(map)))
		, m_finishWaypoints(new QShortcut(this))
	{
		m_ui->setupUi(this);
		m_ui->mapLayout->addWidget(m_canvas);
		m_finishWaypoints->setKeys({QKeySequence(Qt::Key_Return), QKeySequence(Qt::Key_Enter)});
		m_finishWaypoints->setAutoRepeat(false);
		connect(m_finishWaypoints, &QShortcut::activated, this, &ViewerWindow::finishWaypointSelection);

		connect(this, &ViewerWindow::logEntryReady, this, [this](const QString& lightHtml, const QString& darkHtml) {
			const bool darkBackground =
				m_ui->logView->palette().color(QPalette::Base).lightness() < DarkBackgroundThreshold;
			m_ui->logView->appendHtml(darkBackground ? darkHtml : lightHtml);
		}, Qt::QueuedConnection);

		auto* workerTimer = new QTimer(this);
		connect(workerTimer, &QTimer::timeout, this, [this] {
			if (m_session.hasWorkerFailure())
				stopDrive();
		});
		workerTimer->start(WorkerCheckMilliseconds);
		connect(m_ui->help, &QPushButton::clicked, this, &ViewerWindow::showHelp);
		connect(m_ui->setPoints, &QPushButton::clicked, this, [this] { m_session.beginSetting(); });
		connect(m_ui->cancel, &QPushButton::clicked, this, [this] { m_session.cancelSetting(); });
		connect(m_ui->routeOptions, &QComboBox::currentIndexChanged, this, [this](int index) {
			if (index >= 0 && m_session.selectRoute(static_cast<std::size_t>(index)))
				updateRoutePreview();
		});
		connect(m_ui->start, &QPushButton::clicked, this, &ViewerWindow::startDrive);
		connect(m_ui->pause, &QPushButton::clicked, this, [this] {
			if (m_session.state() == State::Paused)
				m_session.resume();
			else
				m_session.requestPause([this] { QCoreApplication::postEvent(this, new ControlEvent(AfterControl::None)); });
		});
		connect(m_ui->stop, &QPushButton::clicked, this, [this] { stopDrive(); });
		connect(m_ui->reset, &QPushButton::clicked, this, &ViewerWindow::resetDrive);
		connect(m_ui->recordButton, &QPushButton::clicked, this, &ViewerWindow::toggleRecording);
		connect(this, &ViewerWindow::csvRecordingFinished,
				this, &ViewerWindow::finishRecording, Qt::QueuedConnection);
		connect(m_ui->noise, &QSlider::valueChanged, this, [this](int value) {
			m_session.setNoise(value);
			m_ui->noiseLabel->setText(QString("GPS noise: %1 m").arg(value));
		});
		connect(m_ui->speed, &QSlider::valueChanged, this, [this](int value) {
			m_session.setSpeed(value);
			m_ui->speedLabel->setText(QString("Speed: %1 m/s").arg(value));
		});
		connect(m_ui->gpsOutage, &QCheckBox::toggled, this, [this](bool enabled) { m_session.setGpsOutage(enabled); });
		m_canvas->setPickHandler([this](nav::geometry::Point point) { pickRoutePoint(point); });
		m_canvas->setClearEndpointsHandler([this] { return m_session.clearEndpoints(); });
		m_session.setStateHandler([this](State) { updateControls(); });
		updateControls();
		nav::logger::applicationLog().setDisplayHandler([this](const nav::logger::Log::DisplayEntry& entry) {
			Q_EMIT logEntryReady(QString::fromStdString(entry.lightHtml), QString::fromStdString(entry.darkHtml));
		});
	}

	// Stops the session before Qt destroys the viewer and its event targets.
	ViewerWindow::~ViewerWindow()
	{
		m_recorder.stop();
		m_recorder.join();
		nav::logger::applicationLog().setDisplayHandler({});
		m_session.shutdown();
	}

	void ViewerWindow::closeEvent(QCloseEvent* event)
	{
		if (m_recordingState == RecordingState::Idle)
		{
			QWidget::closeEvent(event);
			return;
		}

		event->ignore();
		m_closeAfterRecording = true;
		m_recordingState = RecordingState::Finishing;
		m_recorder.stop();
		updateRecordingControls();
	}

	// Handles background route and drive events on the Qt GUI thread.
	bool ViewerWindow::event(QEvent* event)
	{
		if (event->type() == FrameEvent::typeId())
		{
			const auto& data = static_cast<FrameEvent*>(event)->data;
			if (data.driveId == m_driveId)
				displayFrame(data.frame);
			return true;
		}
		if (event->type() == PlanEvent::typeId())
		{
			m_session.finishCalculation(std::move(static_cast<PlanEvent*>(event)->data));
			return true;
		}
		if (event->type() == ControlEvent::typeId())
		{
			m_session.finishControl();
			if (static_cast<ControlEvent*>(event)->data == AfterControl::Reset)
				resetDrive();
			else if (m_session.state() == State::Paused && m_arrived)
				stopDrive(true);
			return true;
		}
		return QWidget::event(event);
	}

	// Updates control availability and status text for the current session state.
	void ViewerWindow::updateControls()
	{
		const auto state = m_session.state();
		const bool choosing = state == State::ChoosingRoute;
		const bool setting = state == State::SettingRoute;
		const bool driving = state == State::Running || state == State::Paused;
		m_ui->stateLabel->setText(stateName(state));
		m_ui->setPoints->setEnabled(state == State::NotStarted || choosing);
		m_ui->setPoints->setChecked(setting);
		m_ui->setPoints->setText(setting ? "Setting Start / Destination…" : "Set Start / Destination");
		m_ui->cancel->setEnabled(setting);
		m_finishWaypoints->setEnabled(setting);
		m_ui->routeOptions->setEnabled(choosing);
		m_ui->start->setEnabled(choosing && !m_session.options().empty());
		m_ui->pause->setEnabled(driving);
		m_ui->pause->setText(state == State::Paused ? "Resume" : "Pause");
		m_ui->stop->setEnabled(driving);
		m_ui->reset->setEnabled(driving || choosing || state == State::Stopped || state == State::SettingRoute);
		updateRecordingControls();
		m_canvas->setCursor(state == State::SettingRoute ? Qt::CrossCursor : Qt::ArrowCursor);

		QString notice;
		switch (state)
		{
		case State::NotStarted:
			notice = "Select Set Start / Destination to begin picking waypoints.";
			break;
		case State::SettingRoute:
			notice = "Click at least two points on the map, then press Enter to plan the route.";
			break;
		case State::Calculating:
			notice = "Calculating routes…";
			break;
		case State::ChoosingRoute:
			notice = m_session.options().size() > 1 ? "Select a route on the map or in the list, then press Start."
													: "One drivable route found. Press Start to follow it.";
			break;
		case State::Starting:
			notice = "Preparing the selected route…";
			break;
		case State::Running:
			notice = "Following the selected route. Stop to plan another drive.";
			break;
		case State::Pausing:
			notice = "Finishing in-flight GPS work…";
			break;
		case State::Paused:
			notice = "Drive paused. Resume to continue from the current position.";
			break;
		case State::Stopping:
			notice = "Stopping acquisition and tracking…";
			break;
		case State::Stopped:
			notice = m_arrived ? "Destination reached. Select Reset to plan another drive."
							   : "Drive stopped. Select Reset to plan another drive.";
			break;
		}
		m_ui->routeStatus->setText(notice);

		const QSignalBlocker blocker(m_ui->routeOptions);
		m_ui->routeOptions->clear();
		for (std::size_t i = 0; i < m_session.options().size(); ++i)
		{
			const auto& option = m_session.options()[i];
			m_ui->routeOptions->addItem(
				QString("%1 · %2 m").arg(i == 0 ? "Shortest" : "Alternate").arg(option.meters, 0, 'f', 0));
		}
		if (!m_session.options().empty())
			m_ui->routeOptions->setCurrentIndex(static_cast<int>(m_session.selectedRoute()));
		updateRoutePreview();
	}

	// Synchronizes displayed paths and endpoint markers with the current plan.
	void ViewerWindow::updateRoutePreview()
	{
		const auto& options = m_session.options();
		m_canvas->setRoutes(options.empty() ? nullptr : options[0].points,
							options.size() < 2 ? nullptr : options[1].points,
							m_session.selectedRoute(),
							m_session.state() == State::ChoosingRoute);
		m_canvas->setWaypoints(m_session.waypoints().positions, m_session.state() != State::SettingRoute);
	}

	// Uses a map click to set an endpoint or choose a displayed alternative.
	void ViewerWindow::pickRoutePoint(nav::geometry::Point point)
	{
		if (m_session.state() == State::SettingRoute)
		{
			if (m_session.pickPoint(point))
				updateControls();
		}
		else if (m_session.state() == State::ChoosingRoute)
		{
			if (const auto index = m_canvas->routeAt(point))
				m_ui->routeOptions->setCurrentIndex(static_cast<int>(*index));
		}
	}

	void ViewerWindow::finishWaypointSelection()
	{
		m_session.finishWaypoints([this](nav::session::RouteSession::PlanResult result) {
			QCoreApplication::postEvent(this, new PlanEvent(std::move(result)));
		});
	}

	// Starts the selected route and posts worker frames to the GUI event queue.
	void ViewerWindow::startDrive()
	{
		if (m_session.state() != State::ChoosingRoute)
			return;
		m_arrived = false;
		m_canvas->clearFrame();
		const auto driveId = ++m_driveId;
		m_session.start([this, driveId](nav::session::DriveFrame frame) {
			QCoreApplication::postEvent(this, new FrameEvent({std::move(frame), driveId}));
		});
	}

	// Stops the session asynchronously and invalidates any stale frame events.
	void ViewerWindow::stopDrive(bool reset)
	{
		const auto after = reset ? AfterControl::Reset : AfterControl::None;
		if (m_session.requestStop([this, after] { QCoreApplication::postEvent(this, new ControlEvent(after)); }))
			++m_driveId;
	}

	// Clears route and display state after the session has stopped its workers.
	void ViewerWindow::resetDrive()
	{
		const auto state = m_session.state();
		if (state == State::Running || state == State::Paused)
		{
			stopDrive(true);
			return;
		}
		if (m_session.resetRoute())
		{
			m_arrived = false;
			m_canvas->clearFrame();
			m_ui->status->clear();
		}
	}

	// Updates the map, status, and logging sinks from one matched drive frame.
	void ViewerWindow::displayFrame(const nav::session::DriveFrame& frame)
	{
		m_canvas->setFrame(frame);
		m_recorder.append(frame);
		if (const auto& match = frame.match)
		{
			m_ui->status->setText(QString("True progress: %1 m | Estimated: %2 m | Remaining: %3 m | Distance to route: %4 m | %5")
									  .arg(frame.progressMeters, 0, 'f', ProgressDecimalPlaces)
									  .arg(match->progressMeters, 0, 'f', ProgressDecimalPlaces)
									  .arg(match->remainingMeters, 0, 'f', ProgressDecimalPlaces)
									  .arg(match->distanceFromRouteMeters, 0, 'f', ProgressDecimalPlaces)
									  .arg(frame.arrived ? "ARRIVED" : (match->offRoute ? "OFF ROUTE" : "ON ROUTE")));
		}
		else if (frame.prediction)
		{
			m_ui->status->setText(QString("True progress: %1 m | Estimated: %2 m | %3")
									  .arg(frame.progressMeters, 0, 'f', ProgressDecimalPlaces)
									  .arg(frame.prediction->progressMeters, 0, 'f', ProgressDecimalPlaces)
									  .arg(frame.arrived ? "ARRIVED" : "DEAD RECKONING"));
		}
		else
		{
			m_ui->status->setText(frame.arrived ? "ARRIVED" : "GPS unavailable | No position estimate");
		}
		if (frame.arrived)
		{
			m_arrived = true;
			stopDrive(true);
		}
	}

	// Displays the route setup and drive controls help dialog.
	void ViewerWindow::showHelp()
	{
		QMessageBox::information(
			this,
			"Plan and drive a route",
			"1. Select Set Start / Destination and click at least two stops in order.\n"
			"   Press Enter to plan. Right-click the map to clear all stops and route choices.\n"
			"2. Compare the routes. Click a colored line or choose it from the route list.\n"
			"3. Press Start to drive the selected route.\n\n"
			"Pause keeps your place. Stop ends the drive. Reset clears the route so you can set new points.");
	}

	void ViewerWindow::toggleRecording()
	{
		if (m_recordingState == RecordingState::Finishing)
			return;

		if (m_recordingState == RecordingState::Recording)
		{
			m_recordingState = RecordingState::Finishing;
			m_recorder.stop();
			updateRecordingControls();
			return;
		}

		QFileDialog dialog(this, "Record GPS readings");
		dialog.setAcceptMode(QFileDialog::AcceptSave);
		dialog.setNameFilter("CSV files (*.csv)");
		dialog.setDefaultSuffix("csv");
		dialog.selectFile("RouteRecording.csv");
		if (dialog.exec() != QDialog::Accepted)
			return;

		const auto path = std::filesystem::path(dialog.selectedFiles().constFirst().toStdU16String());
		if (!m_recorder.start(path, [this](bool success) { Q_EMIT csvRecordingFinished(success); }))
		{
			finishRecording(false);
			return;
		}
		m_recordingState = RecordingState::Recording;
		updateRecordingControls();
	}

	void ViewerWindow::updateRecordingControls()
	{
		const auto state = m_session.state();
		const bool driving = state == State::Running || state == State::Paused;
		switch (m_recordingState)
		{
		case RecordingState::Idle:
			m_ui->recordButton->setText("Record CSV");
			m_ui->recordButton->setEnabled(driving);
			break;
		case RecordingState::Recording:
			m_ui->recordButton->setText("Stop Recording");
			m_ui->recordButton->setEnabled(true);
			break;
		case RecordingState::Finishing:
			m_ui->recordButton->setText("Finishing Recording…");
			m_ui->recordButton->setEnabled(false);
			break;
		}
	}

	void ViewerWindow::finishRecording(bool success)
	{
		m_recorder.join();
		m_recordingState = RecordingState::Idle;
		updateRecordingControls();
		if (!success)
			nav::logger::error("CSV recording failed. The file may be incomplete.");

		if (m_closeAfterRecording)
		{
			close();
			return;
		}
		if (!success)
			QMessageBox::warning(this, "CSV recording failed", "The file could not be completed.");
	}
}    // namespace nav::viewer
