#pragma once

#include "nav/Routing/RoutePlanner.h"
#include "nav/Simulation/SimulatedGnssSource.h"
#include "nav/Session/TrackingWorker.h"

namespace nav::session
{
	/// Owns the planned routes and the lifetime of one drive. Commands run on the owning thread.
	class RouteSession
	{
	public:
		enum class State
		{
			NotStarted,
			SettingRoute,
			Calculating,
			ChoosingRoute,
			Starting,
			Running,
			Pausing,
			Paused,
			Stopping,
			Stopped
		};

		struct PlanResult
		{
			std::vector<nav::routing::RouteOption> options;    // Routes produced by the background planner for the viewer.
			bool failed = false;                               // Whether planning already logged an error.
		};

		using PlanHandler = std::function<void(PlanResult)>;
		using StateHandler = std::function<void(State)>;
		using ControlHandler = std::function<void()>;

		/// Builds the graph and saves the map origin for route planning and drive setup.
		explicit RouteSession(const nav::routing::StreetMap& map);
		/// Stops and joins the session threads before releasing their resources.
		~RouteSession();
		/// Registers the viewer callback for session state changes.
		void setStateHandler(StateHandler handler);
		/// Clears the prior plan and enables picking route endpoints.
		bool beginSetting();
		/// Discards partial endpoint selection and returns to the initial state.
		bool cancelSetting();
		/// Appends a snapped waypoint without starting a search.
		bool pickPoint(nav::geometry::Point point);
		/// Plans through the selected waypoints in click order.
		bool finishWaypoints(PlanHandler completed);
		/// Clears both endpoints and route alternatives while keeping point selection active.
		bool clearEndpoints();
		// Deliver this on the owning thread after the asynchronous planner posts its result.
		void finishCalculation(PlanResult result);
		/// Records which candidate the viewer selected before starting the drive.
		bool selectRoute(std::size_t index);
		/// Creates the queue and starts the acquisition and tracking threads for the selected route.
		bool start(TrackingWorker::FrameHandler handler);
		/// Requests a pause and signals completion after in-flight work finishes.
		bool requestPause(ControlHandler completed);
		/// Resumes the paused drive and lets tracking drain preserved fixes first.
		bool resume();
		/// Requests a stop and signals completion after both workers have exited.
		bool requestStop(ControlHandler completed);
		/// Applies a completed pause or stop on the owning thread.
		void finishControl();
		/// Clears endpoint selection, route candidates, or the previous stopped drive.
		bool resetRoute();
		/// Cancels background planning and tears down the drive workers.
		void shutdown();
		/// Updates the speed used to advance the simulated vehicle at each sample.
		void setSpeed(int metersPerSecond);
		/// Updates the noise amplitude used for simulated GPS fixes.
		void setNoise(int meters);
		/// Controls the northward GPS offset used in the simulation.
		void setOffset(bool enabled);

		/// Returns the current drive state for UI control and status updates.
		State state() const;
		/// Detects when a worker closes the queue during an active drive.
		bool hasWorkerFailure() const;
		/// Returns the route candidates used by the viewer preview and selector.
		const std::vector<nav::routing::RouteOption>& options() const;
		/// Returns the index of the candidate highlighted in the viewer.
		std::size_t selectedRoute() const;
		/// Returns the selected waypoints in click order.
		const nav::routing::Waypoint& waypoints() const;

	private:
		/// Changes the session state and notifies the registered UI callback.
		void transition(State next);
		/// Clears route endpoints, alternatives, selection, and route status.
		void clearPlan();
		/// Closes the queue and joins both drive workers before releasing it.
		void stopWorkers();
		/// Starts a control operation without doing its wait on the owning thread.
		bool startControl(State pending, ControlHandler completed);

		const nav::routing::RoadGraph m_graph;         // Road network used to snap clicks and plan routes.
		const nav::geometry::Point m_originDegrees;    // Map origin used by both acquisition and tracking projections.
		State m_state = State::NotStarted;             // Current lifecycle stage used to gate session commands.
		StateHandler m_stateChanged;                   // Callback that updates the viewer when the lifecycle stage changes.
		nav::routing::Waypoint m_waypoints;    // Snapped stops in click order.
		std::vector<nav::routing::RouteOption> m_options;                // Planned route alternatives available to the viewer.
		std::size_t m_selected = 0;                 // Index of the candidate used for preview and drive startup.
		int m_speed = 12;                           // Selected speed retained across drives and forwarded to acquisition.
		int m_noise = 8;                            // Selected GPS noise retained across drives and forwarded to acquisition.
		bool m_offset = false;                      // Selected GPS offset setting retained across drives.
		std::unique_ptr<ProtectedQueue> m_queue;    // Session-owned handoff shared by both drive workers.
		logger::PositionLogger m_positionLogger;  // Retains one logger across all drives in this session.
		std::unique_ptr<simulation::SimulatedGnssSource>
			m_source;                                // Owns the simulated GNSS acquisition thread for the current drive.
		std::unique_ptr<TrackingWorker> m_worker;    // Owns the tracking consumer thread for the current drive.
		std::jthread m_calculation;                  // Plans alternatives asynchronously so route setup stays responsive.
		std::jthread m_control;                      // Waits for pause or stop without blocking the session owner.
	};
}    // namespace nav::session
