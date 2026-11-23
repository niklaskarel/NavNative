#pragma once

#include "nav/Routing/LocalProjection.h"
#include "nav/Geometry/NavigationTypes.h"
#include "nav/Logger/PositionLogger.h"
#include "nav/Session/ProtectedQueue.h"
#include "nav/Tracking/RouteTracker.h"

#include <functional>
#include <thread>

namespace nav::session
{
	/// Consumes receiver messages and delivers matched frames.
	class TrackingWorker
	{
	public:
		using FrameHandler = std::function<void(DriveFrame)>;

		/// Prepares route matching and geographic conversion for the queue consumer.
		TrackingWorker(geometry::SharedRoute route, geometry::Point originDegrees, ProtectedQueue& queue,
					   logger::PositionLogger& positionLogger);
		/// Starts the consumer thread that delivers matched frames to the viewer.
		void start(FrameHandler handler);

	private:
		/// Decodes queued NMEA, matches valid fixes, and delivers completed drive frames.
		void process(std::stop_token stopToken, const FrameHandler& handler);

		tracking::RouteTracker m_tracker;         // Matches projected fixes and retains history across packets.
		routing::LocalProjection m_projection;    // Converts decoded geographic coordinates to local route meters.
		ProtectedQueue& m_queue;                  // Borrowed queue from which the consumer reads GNSS packets.
		logger::PositionLogger& m_positionLogger; // Session-owned position logger shared across drives.
		std::jthread m_thread;                    // Runs decoding and route matching for the active drive.
	};
}    // namespace nav::session
