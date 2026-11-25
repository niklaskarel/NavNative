#pragma once

#include "nav/Logger/CsvRecorder.h"
#include "nav/Session/RouteSession.h"

#include <QWidget>

#include <memory>

class QCloseEvent;
class QShortcut;

namespace Ui
{
	class ViewerWindow;
}

namespace nav::viewer
{
	class MapCanvas;

	/// Connects the route session to Qt controls and displays completed frames.
	class ViewerWindow : public QWidget
	{
		Q_OBJECT

	public:
		/// Loads the Designer form and connects its controls to the session and map.
		explicit ViewerWindow(nav::routing::StreetMap map);
		/// Stops the session before Qt destroys the viewer and its event targets.
		~ViewerWindow() override;

	Q_SIGNALS:
		void logEntryReady(const QString& lightHtml, const QString& darkHtml);
		void csvRecordingFinished(bool success);

	protected:
		/// Handles background route and drive events on the Qt GUI thread.
		bool event(QEvent* event) override;
		void closeEvent(QCloseEvent* event) override;

	private:
		/// Updates control availability and status text for the current session state.
		void updateControls();
		/// Synchronizes displayed paths and endpoint markers with the current plan.
		void updateRoutePreview();
		/// Uses a map click to set an endpoint or choose a displayed alternative.
		void pickRoutePoint(nav::geometry::Point point);
		/// Submits the selected stops to the background planner.
		void finishWaypointSelection();
		/// Starts the selected route and posts worker frames to the GUI event queue.
		void startDrive();
		/// Stops the session and optionally clears the route after the workers exit.
		void stopDrive(bool reset = false);
		/// Clears the selected route or requests a stop followed by a reset.
		void resetDrive();
		/// Updates the map, status, and logging sinks from one matched drive frame.
		void displayFrame(const nav::session::DriveFrame& frame);
		/// Displays the route setup and drive controls help dialog.
		void showHelp();
		/// Prompts for a CSV path or stops an existing recording.
		void toggleRecording();
		/// Updates the button after a recorder or drive state change.
		void updateRecordingControls();
		/// Handles the writer result on the GUI thread.
		void finishRecording(bool success);

		enum class RecordingState { Idle, Recording, Finishing };

		nav::logger::CsvRecorder m_recorder;
		RecordingState m_recordingState = RecordingState::Idle;
		bool m_closeAfterRecording = false;
		nav::session::RouteSession m_session;      // Owns route planning, drive state, and background workers for this window.
		std::unique_ptr<Ui::ViewerWindow> m_ui;    // Holds the controls generated from the Designer form.
		MapCanvas* m_canvas;                       // Draws the map and forwards route clicks to the viewer.
		QShortcut* m_finishWaypoints;              // Enabled only while collecting waypoints.
		std::size_t m_driveId = 0;                 // Identifies the active drive so late events can be discarded.
		bool m_arrived = false;                    // Selects the arrival message when the drive stops at its destination.
	};
}    // namespace nav::viewer
