#include "nav/Session/RouteSession.h"

#include "nav/Logger/Log.h"

#include <exception>
#include <utility>

namespace nav::session
{
	namespace
	{
		constexpr std::size_t GnssQueueCapacity = 16;
	}

	// Builds the graph and saves the map origin for route planning and drive setup.
	RouteSession::RouteSession(const nav::routing::StreetMap& map)
		: m_graph(map)
		, m_originDegrees(map.originDegrees)
		, m_positionLogger(logger::applicationLog())
	{
	}

	// Stops and joins the session threads before releasing their resources.
	RouteSession::~RouteSession()
	{
		shutdown();
	}

	// Registers the viewer callback for session state changes.
	void RouteSession::setStateHandler(StateHandler handler)
	{
		m_stateChanged = std::move(handler);
	}

	// Clears the prior plan and enables picking route endpoints.
	bool RouteSession::beginSetting()
	{
		if (m_state != State::NotStarted && m_state != State::ChoosingRoute)
			return false;
		clearPlan();
		transition(State::SettingRoute);
		return true;
	}

	// Discards partial endpoint selection and returns to the initial state.
	bool RouteSession::cancelSetting()
	{
		if (m_state != State::SettingRoute)
			return false;
		clearPlan();
		transition(State::NotStarted);
		return true;
	}

	bool RouteSession::pickPoint(nav::geometry::Point point)
	{
		if (m_state != State::SettingRoute)
			return false;
		const auto node = m_graph.nearestNode(point);
		if (!m_waypoints.nodeIds.empty() && m_waypoints.nodeIds.back() == node)
			return false;

		const auto position = m_graph.position(node);
		m_waypoints.nodeIds.push_back(node);
		try
		{
			m_waypoints.positions.push_back(position);
		}
		catch (...)
		{
			m_waypoints.nodeIds.pop_back();
			throw;
		}
		return true;
	}

	bool RouteSession::finishWaypoints(PlanHandler completed)
	{
		if (m_state != State::SettingRoute || m_waypoints.nodeIds.size() < 2 || !completed)
			return false;

		transition(State::Calculating);
		try
		{
			m_calculation = std::jthread(
				[this, stops = m_waypoints.nodeIds, completed = std::move(completed)](std::stop_token stop) {
					PlanResult result;
					try
					{
						result.options = nav::routing::RoutePlanner(m_graph).routeOptions(stops, stop);
					}
					catch (const std::exception& error)
					{
						result.failed = true;
						nav::logger::error(error.what());
					}
					catch (...)
					{
						result.failed = true;
						nav::logger::error("Unexpected route planning failure");
					}
					try
					{
						if (!stop.stop_requested())
							completed(std::move(result));
					}
					catch (const std::exception& error)
					{
						nav::logger::error(error.what());
					}
					catch (...)
					{
						nav::logger::error("Could not deliver the route calculation result");
					}
				});
		}
		catch (const std::exception& error)
		{
			nav::logger::error(error.what());
			finishCalculation({{}, true});
			return false;
		}
		return true;
	}

	// Clears both markers and the old route choices before refreshing the setup view.
	bool RouteSession::clearEndpoints()
	{
		if ((m_state != State::SettingRoute && m_state != State::ChoosingRoute) || m_waypoints.nodeIds.empty())
			return false;
		clearPlan();
		transition(State::SettingRoute);
		return true;
	}

	// Applies the background planning result on the owning thread.
	void RouteSession::finishCalculation(PlanResult result)
	{
		if (m_state != State::Calculating)
			return;
		if (m_calculation.joinable())
			m_calculation.join();
		m_options = std::move(result.options);
		m_selected = 0;
		if (m_options.empty())
		{
			if (!result.failed)
				nav::logger::warning("The selected waypoints could not be connected. Right-click to clear them and try again.");
			transition(State::SettingRoute);
			return;
		}
		transition(State::ChoosingRoute);
	}

	// Records which candidate the viewer selected before starting the drive.
	bool RouteSession::selectRoute(std::size_t index)
	{
		if (m_state != State::ChoosingRoute || index >= m_options.size())
			return false;
		m_selected = index;
		return true;
	}

	// Creates the queue and starts the acquisition and tracking threads for the selected route.
	bool RouteSession::start(TrackingWorker::FrameHandler handler)
	{
		if (m_state != State::ChoosingRoute || m_selected >= m_options.size())
			return false;
		transition(State::Starting);
		try
		{
			const auto& route = m_options[m_selected].points;
			if (!route || route->size() < 2 || !(m_options[m_selected].meters > 0))
			{
				nav::logger::error("Selected route has no usable geometry");
				transition(State::Stopping);
				transition(State::Stopped);
				return false;
			}
			m_queue = std::make_unique<ProtectedQueue>(GnssQueueCapacity);
			m_source = std::make_unique<simulation::SimulatedGnssSource>(route, m_originDegrees);
			m_worker = std::make_unique<TrackingWorker>(route, m_originDegrees, *m_queue, m_positionLogger);
			m_source->setSpeed(m_speed);
			m_source->setNoise(m_noise);
			m_source->setOffset(m_offset);
			m_worker->start(std::move(handler));
			m_source->start([queue = m_queue.get()](std::stop_token token) { return queue->waitToProduce(token); },
							[queue = m_queue.get()](simulation::GnssPacket&& packet) { queue->push(std::move(packet)); },
							[queue = m_queue.get()] { queue->stop(); });
			m_queue->resume();
		}
		catch (const std::exception& error)
		{
			nav::logger::error(error.what());
			transition(State::Stopping);
			stopWorkers();
			transition(State::Stopped);
			return false;
		}
		transition(State::Running);
		return true;
	}

	// Starts waiting for a pause while preserving queued fixes.
	bool RouteSession::requestPause(ControlHandler completed)
	{
		if (m_state != State::Running)
			return false;
		return startControl(State::Pausing, std::move(completed));
	}

	// Resumes the paused drive and lets tracking drain preserved fixes first.
	bool RouteSession::resume()
	{
		if (m_state != State::Paused)
			return false;
		m_queue->resume();
		transition(State::Running);
		return true;
	}

	// Starts shutting down the drive workers and discarding pending fixes.
	bool RouteSession::requestStop(ControlHandler completed)
	{
		if (m_state != State::Running && m_state != State::Paused)
			return false;
		return startControl(State::Stopping, std::move(completed));
	}

	// Runs the blocking queue operation and posts its completion through the caller's callback.
	bool RouteSession::startControl(State pending, ControlHandler completed)
	{
		try
		{
			m_control = std::jthread([this, pending, completed = std::move(completed)] {
				try
				{
					if (pending == State::Pausing)
						m_queue->pause();
					else
						stopWorkers();
					completed();
				}
				catch (const std::exception& error)
				{
					nav::logger::error(error.what());
				}
				catch (...)
				{
					nav::logger::error("Unexpected drive control failure");
				}
			});
		}
		catch (const std::exception& error)
		{
			nav::logger::error(error.what());
			return false;
		}
		transition(pending);
		return true;
	}

	// Finalizes the drive state after the background operation reports completion.
	void RouteSession::finishControl()
	{
		if (m_state != State::Pausing && m_state != State::Stopping)
			return;
		m_control.join();
		transition(m_state == State::Pausing ? State::Paused : State::Stopped);
	}

	// Clears the current route setup or the previous stopped drive.
	bool RouteSession::resetRoute()
	{
		if (m_state != State::Stopped && m_state != State::SettingRoute && m_state != State::ChoosingRoute)
			return false;
		clearPlan();
		transition(State::NotStarted);
		return true;
	}

	// Cancels background planning and tears down the drive workers.
	void RouteSession::shutdown()
	{
		m_calculation.request_stop();
		if (m_calculation.joinable())
			m_calculation.join();
		if (m_control.joinable())
			m_control.join();
		stopWorkers();
		m_state = State::Stopped;
	}

	// Closes the queue and joins both drive workers before releasing it.
	void RouteSession::stopWorkers()
	{
		if (m_queue)
			m_queue->stop();
		m_source.reset();
		m_worker.reset();
		m_queue.reset();
	}

	// Updates the speed used to advance the simulated vehicle at each sample.
	void RouteSession::setSpeed(int metersPerSecond)
	{
		m_speed = metersPerSecond;
		if (m_state != State::Stopping && m_source)
			m_source->setSpeed(m_speed);
	}

	// Updates the noise amplitude used for simulated GPS fixes.
	void RouteSession::setNoise(int meters)
	{
		m_noise = meters;
		if (m_state != State::Stopping && m_source)
			m_source->setNoise(m_noise);
	}

	// Controls the northward GPS offset used in the simulation.
	void RouteSession::setOffset(bool enabled)
	{
		m_offset = enabled;
		if (m_state != State::Stopping && m_source)
			m_source->setOffset(m_offset);
	}

	// Returns the current drive state for UI control and status updates.
	RouteSession::State RouteSession::state() const
	{
		return m_state;
	}

	// Detects when a worker closes the queue during an active drive.
	bool RouteSession::hasWorkerFailure() const
	{
		return (m_state == State::Running || m_state == State::Paused) && m_queue->isStopped();
	}

	// Returns the route candidates used by the viewer preview and selector.
	const std::vector<nav::routing::RouteOption>& RouteSession::options() const
	{
		return m_options;
	}

	// Returns the index of the candidate highlighted in the viewer.
	std::size_t RouteSession::selectedRoute() const
	{
		return m_selected;
	}

	const nav::routing::Waypoint& RouteSession::waypoints() const
	{
		return m_waypoints;
	}

	// Changes the session state and notifies the registered UI callback.
	void RouteSession::transition(State next)
	{
		m_state = next;
		if (m_stateChanged)
			m_stateChanged(next);
	}

	// Clears route endpoints, alternatives, selection, and route status.
	void RouteSession::clearPlan()
	{
		m_waypoints = {};
		m_options.clear();
		m_selected = 0;
	}
}    // namespace nav::session
