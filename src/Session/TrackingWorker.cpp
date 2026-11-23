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
		std::optional<std::chrono::sys_time<std::chrono::milliseconds>> firstFixTime;
		const auto clockOrigin = std::chrono::steady_clock::now();
		while (auto packet = m_queue.waitAndPop(stopToken))
		{
			if (const auto decoded = nav::gnss::parseRmc(packet->sentence))
			{
				if (!firstFixTime)
					firstFixTime = decoded->measuredAt;
				packet->sample.gps = m_projection.project(decoded->degrees);
				const tracking::GpsFix fix{packet->sample.gps, clockOrigin + (decoded->measuredAt - *firstFixTime)};
				if (auto match = m_tracker.track(fix))
				{
					m_positionLogger.log(packet->sample.truth);
					handler(DriveFrame{std::move(packet->sample), *match, packet->arrived});
				}
			}
			else
				nav::logger::warning("Invalid NMEA RMC sentence discarded");
			m_queue.complete();
		}
	}
}    // namespace nav::session
