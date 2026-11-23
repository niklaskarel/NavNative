#pragma once

#include "nav/Simulation/DriveSimulation.h"
#include "nav/Routing/LocalProjection.h"
#include "nav/Geometry/NavigationTypes.h"

#include <atomic>
#include <functional>
#include <thread>

namespace nav::simulation
{
	/// Generates simulated receiver messages on its own thread at 10 Hz.
	class SimulatedGnssSource
	{
	public:
		using WaitForInput = std::function<bool(std::stop_token)>;
		using Publish = std::function<void(GnssPacket&&)>;

		/// Prepares the simulator and geographic projection for acquisition.
		SimulatedGnssSource(geometry::SharedRoute route, geometry::Point originDegrees);
		/// Starts the acquisition thread with callbacks for queue access and failure handling.
		void start(WaitForInput waitForInput, Publish publish, std::function<void()> onFailure);

		/// Updates the speed used to advance the simulated vehicle at each sample.
		void setSpeed(int metersPerSecond);
		/// Updates the noise amplitude used for simulated GPS fixes.
		void setNoise(int meters);
		/// Controls the northward GPS offset used in the simulation.
		void setOffset(bool enabled);

	private:
		/// Paces the producer at 10 Hz and publishes timestamped receiver packets.
		void acquire(std::stop_token stopToken, const WaitForInput& waitForInput, const Publish& publish);

		DriveSimulation m_simulation;             // Advances the vehicle and generates truth and simulated GPS samples.
		routing::LocalProjection m_projection;    // Converts local simulated positions to geographic coordinates for NMEA.
		std::atomic<int> m_speed = 12;            // Atomic speed setting read by the acquisition thread in meters per second.
		std::atomic<int> m_noise = 8;             // Atomic GPS noise setting read by acquisition in meters.
		std::atomic<bool> m_offsetEnabled = false;    // Atomic switch for the simulated northward GPS bias.
		std::jthread m_thread;                        // Runs the timed receiver simulation and publishes packets.
	};
}    // namespace nav::simulation
