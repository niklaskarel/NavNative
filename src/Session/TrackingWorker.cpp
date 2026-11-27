#include "nav/Session/TrackingWorker.h"

#include "nav/Logger/Log.h"
#include "nav/Gnss/NmeaRmc.h"

#include <chrono>
#include <exception>
#include <utility>

namespace nav::session
{
	// Prepares route matching and geographic conversion for the queue consumer.
	TrackingWorker::TrackingWorker(geometry::SharedRoute route, geometry::Point originDegrees, ProtectedQueue& queue,
								   logger::PositionLogger& positionLogger)
		: m_tracker(std::move(route))
		, m_projection(originDegrees)
		, m_queue(queue)
		, m_positionLogger(positionLogger)
	{
	}

	// Starts the consumer thread that delivers matched frames to the viewer.
	void TrackingWorker::start(FrameHandler handler)
	{
		m_thread = std::jthread([this, handler = std::move(handler)](std::stop_token token) {
			try
			{
				process(token, handler);
			}
			catch (const std::exception& error)
			{
				nav::logger::error(error.what());
				m_queue.complete();
				m_queue.stop();
			}
			catch (...)
			{
				nav::logger::error("Unexpected tracking worker failure");
				m_queue.complete();
				m_queue.stop();
			}
		});
	}

	// Decodes queued NMEA, matches valid fixes, and delivers completed drive frames.
	void TrackingWorker::process(std::stop_token stopToken, const FrameHandler& handler)
	{
		m_positionLogger.reset();
		std::optional<std::chrono::sys_time<std::chrono::milliseconds>> firstSampleTime;
		const auto clockOrigin = std::chrono::steady_clock::now();
		while (auto packet = m_queue.waitAndPop(stopToken))
		{
			if (!firstSampleTime)
				firstSampleTime = packet->timestamp;

			DriveFrame frame{};
			frame.progressMeters = packet->sample.progressMeters;
			frame.truth = packet->sample.truth;
			frame.arrived = packet->arrived;
			if (!packet->sentence)
			{
				const auto timestamp = clockOrigin + (packet->timestamp - *firstSampleTime);
				if (const auto progress = m_tracker.predictProgress(timestamp, packet->simulatedSpeed))
					frame.prediction = RoutePrediction{m_tracker.pointAt(*progress), *progress};
			}
			else if (const auto decoded = nav::gnss::parseRmc(*packet->sentence))
			{
				frame.gps = m_projection.project(decoded->degrees);
				const auto timestamp = clockOrigin + (decoded->measuredAt - *firstSampleTime);
				frame.match = m_tracker.track({*frame.gps, timestamp});
			}
			else
				nav::logger::warning("Invalid NMEA RMC sentence discarded");
			if (!packet->sentence || frame.match || frame.arrived)
			{
				m_positionLogger.log(frame.truth);
				handler(std::move(frame));
			}
			m_queue.complete();
		}
	}
}    // namespace nav::session
