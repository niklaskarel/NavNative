#include "nav/Simulation/SimulatedGnssSource.h"

#include "nav/Logger/Log.h"
#include "nav/Gnss/NmeaRmc.h"

#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <utility>

namespace nav::simulation
{
	namespace
	{
		constexpr auto SampleInterval = std::chrono::milliseconds(100);
		constexpr double NorthOffsetMeters = 60;
	}

	// Prepares the simulator and geographic projection for acquisition.
	SimulatedGnssSource::SimulatedGnssSource(geometry::SharedRoute route, geometry::Point originDegrees)
		: m_simulation(std::move(route))
		, m_projection(originDegrees)
	{
	}

	// Starts the acquisition thread with callbacks for queue access and failure handling.
	void SimulatedGnssSource::start(WaitForInput waitForInput, Publish publish, std::function<void()> onFailure)
	{
		m_thread = std::jthread([this, wait = std::move(waitForInput), send = std::move(publish), fail = std::move(onFailure)](
									std::stop_token token) {
			try
			{
				acquire(token, wait, send);
				return;
			}
			catch (const std::exception& error)
			{
				nav::logger::error(error.what());
			}
			catch (...)
			{
				nav::logger::error("Unexpected GNSS acquisition failure");
			}
			if (fail)
				fail();
		});
	}

	// Updates the speed used to advance the simulated vehicle at each sample.
	void SimulatedGnssSource::setSpeed(int metersPerSecond)
	{
		m_speed.store(metersPerSecond, std::memory_order_relaxed);
	}

	// Updates the noise amplitude used for simulated GPS fixes.
	void SimulatedGnssSource::setNoise(int meters)
	{
		m_noise.store(meters, std::memory_order_relaxed);
	}

	// Controls the northward GPS offset used in the simulation.
	void SimulatedGnssSource::setOffset(bool enabled)
	{
		m_offsetEnabled.store(enabled, std::memory_order_relaxed);
	}

	// Paces the producer at 10 Hz and publishes timestamped receiver packets.
	void SimulatedGnssSource::acquire(std::stop_token stopToken, const WaitForInput& waitForInput, const Publish& publish)
	{
		const auto utc = std::chrono::floor<std::chrono::milliseconds>(std::chrono::system_clock::now());
		std::condition_variable_any timer;
		std::mutex timerMutex;
		auto fixTime = utc;
		bool first = true;
		while (waitForInput(stopToken))
		{
			const auto speed = m_speed.load(std::memory_order_relaxed);
			const auto arrived = !first && !m_simulation.advance(speed * std::chrono::duration<double>(SampleInterval).count());
			auto sample = m_simulation.sample(m_noise.load(std::memory_order_relaxed),
											  m_offsetEnabled.load(std::memory_order_relaxed) ? NorthOffsetMeters : 0);
			// Replay time advances with samples so a UI pause does not widen the match window.
			auto sentence = nav::gnss::encodeRmc({m_projection.unproject(sample.gps), fixTime});
			publish({std::move(sample), std::move(sentence), arrived});
			if (arrived)
				break;
			first = false;
			fixTime += SampleInterval;
			std::unique_lock lock(timerMutex);
			timer.wait_until(lock, stopToken, std::chrono::steady_clock::now() + SampleInterval, [] { return false; });
		}
	}

}    // namespace nav::simulation
