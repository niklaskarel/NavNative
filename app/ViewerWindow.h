#pragma once

#include "nav/Logger/CsvRecorder.h"
#include "nav/Session/RouteSession.h"

#include <QWidget>

#include <memory>

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

	protected:
		/// Handles background route and drive events on the Qt GUI thread.
		bool event(QEvent* event) override;

	private:
		/// Updates control availability and status text for the current session state.
		void updateControls();
		/// Synchronizes displayed paths and endpoint markers with the current plan.
		void updateRoutePreview();
		/// Uses a map click to set an endpoint or choose a displayed alternative.
		void pickRoutePoint(nav::geometry::Point point);
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
		/// Stops CSV recording and reports the write failure to the user and log.
		void recordingFailed();

		nav::logger::CsvRecorder m_recorder;             // Writes incoming frames to the CSV selected by the user.
		nav::session::RouteSession m_session;      // Owns route planning, drive state, and background workers for this window.
		std::unique_ptr<Ui::ViewerWindow> m_ui;    // Holds the controls generated from the Designer form.
		MapCanvas* m_canvas;                       // Draws the map and forwards route clicks to the viewer.
		std::size_t m_driveId = 0;                 // Identifies the active drive so late events can be discarded.
		bool m_arrived = false;                    // Selects the arrival message when the drive stops at its destination.
	};
}    // namespace nav::viewer
