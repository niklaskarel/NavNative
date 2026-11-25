#include "nav/Session/RouteSession.h"

#include "nav/Session/ProtectedQueue.h"

#include <gtest/gtest.h>

#include <future>

namespace
{
	using namespace std::chrono_literals;
	using State = nav::session::RouteSession::State;

	// Builds a small directed street graph with alternatives for routing and session tests.
	nav::routing::StreetMap sampleMap()
	{
		nav::routing::StreetMap map;
		map.streets = {
			{{1, 2, 3}, {{0, 0}, {10, 0}, {20, 0}}, true},
			{{1, 4, 3}, {{0, 0}, {10, 10}, {20, 0}}, true},
			{{1, 5, 3}, {{0, 0}, {10, 20}, {20, 0}}, true},
		};
		return map;
	}
}

// Verifies candidate order, geometry, and one-way reachability.
TEST(RoutePlannerTest, ReturnsTwoShortestDistinctOptions)
{
	const nav::routing::RoadGraph graph(sampleMap());
	const nav::routing::RoutePlanner planner(graph);
	const auto options = planner.routeOptions(1, 3);
	ASSERT_EQ(options.size(), 2);
	EXPECT_DOUBLE_EQ(options[0].meters, 20);
	EXPECT_NEAR(options[1].meters, 2 * std::hypot(10, 10), 1e-9);
	EXPECT_DOUBLE_EQ((*options[1].points)[1].y, 10);
	EXPECT_TRUE(planner.routeOptions(3, 1).empty());
	EXPECT_EQ(planner.routeOptions(1, 2).size(), 1);
}

// Verifies explicit submission, required intermediate stops, and clearing before and after planning.
TEST(RouteSessionTest, PlansThroughSelectedWaypointsAndClearsThem)
{
	std::promise<nav::session::RouteSession::PlanResult> initial;
	auto plan = initial.get_future();
	nav::session::RouteSession session(sampleMap());

	ASSERT_TRUE(session.beginSetting());
	const auto completed = [&](auto result) { initial.set_value(std::move(result)); };
	ASSERT_TRUE(session.pickPoint({0, 0}));
	EXPECT_FALSE(session.finishWaypoints(completed));
	EXPECT_FALSE(session.pickPoint({0, 0}));
	ASSERT_TRUE(session.pickPoint({10, 10}));
	ASSERT_TRUE(session.pickPoint({20, 0}));
	EXPECT_EQ(session.state(), State::SettingRoute);
	EXPECT_EQ(session.waypoints().nodeIds, (std::vector<nav::routing::NodeId>{1, 4, 3}));
	ASSERT_EQ(session.waypoints().positions.size(), 3);
	EXPECT_DOUBLE_EQ(session.waypoints().positions[1].y, 10);
	ASSERT_TRUE(session.finishWaypoints(completed));
	EXPECT_EQ(session.state(), State::Calculating);
	EXPECT_FALSE(session.clearEndpoints());
	ASSERT_EQ(plan.wait_for(2s), std::future_status::ready);
	session.finishCalculation(plan.get());
	ASSERT_EQ(session.state(), State::ChoosingRoute);
	ASSERT_FALSE(session.options().empty());
	const auto& route = *session.options().front().points;
	ASSERT_EQ(route.size(), 3);
	EXPECT_DOUBLE_EQ(route[1].y, 10);

	ASSERT_TRUE(session.clearEndpoints());
	EXPECT_EQ(session.state(), State::SettingRoute);
	EXPECT_TRUE(session.waypoints().nodeIds.empty());
	EXPECT_TRUE(session.waypoints().positions.empty());
	EXPECT_TRUE(session.options().empty());
	EXPECT_EQ(session.selectedRoute(), 0);
	EXPECT_FALSE(session.start([](nav::session::DriveFrame) {}));

	ASSERT_TRUE(session.pickPoint({0, 0}));
	ASSERT_TRUE(session.pickPoint({10, 0}));
	ASSERT_TRUE(session.pickPoint({0, 0}));
	ASSERT_EQ(session.waypoints().nodeIds.size(), 3);
	ASSERT_TRUE(session.clearEndpoints());
	EXPECT_TRUE(session.waypoints().nodeIds.empty());
	EXPECT_TRUE(session.waypoints().positions.empty());
}

// Verifies the drive lifecycle and delivery of the first simulated frame.
TEST(RouteSessionTest, RequiresSetupAndStopsBeforeAllowingNewRoute)
{
	std::vector<State> transitions;
	std::promise<nav::session::DriveFrame> firstFrame;
	std::promise<void> releaseFrame;
	std::promise<void> paused;
	std::promise<void> stopped;
	nav::session::RouteSession session(sampleMap());
	session.setStateHandler([&](State state) { transitions.push_back(state); });
	std::promise<nav::session::RouteSession::PlanResult> promise;
	auto plan = promise.get_future();
	// Delivers the planner result to the waiting test through its promise.
	auto completed = [&](nav::session::RouteSession::PlanResult result) { promise.set_value(std::move(result)); };
	EXPECT_FALSE(session.pickPoint({0, 0}));
	EXPECT_FALSE(session.start([](nav::session::DriveFrame) {}));
	ASSERT_TRUE(session.beginSetting());
	ASSERT_TRUE(session.pickPoint({0, 0}));
	ASSERT_TRUE(session.pickPoint({20, 0}));
	ASSERT_TRUE(session.finishWaypoints(completed));
	EXPECT_EQ(session.state(), State::Calculating);
	ASSERT_EQ(plan.wait_for(2s), std::future_status::ready);
	session.finishCalculation(plan.get());
	ASSERT_EQ(session.options().size(), 2);
	ASSERT_TRUE(session.selectRoute(1));
	auto frame = firstFrame.get_future();
	ASSERT_TRUE(session.start(
		[&firstFrame, release = releaseFrame.get_future().share(), delivered = false](nav::session::DriveFrame next) mutable {
			if (delivered)
				return;
			delivered = true;
			firstFrame.set_value(std::move(next));
			release.wait_for(2s);
		}));
	ASSERT_EQ(frame.wait_for(2s), std::future_status::ready);
	EXPECT_DOUBLE_EQ(frame.get().progressMeters, 0);
	EXPECT_FALSE(session.beginSetting());
	EXPECT_FALSE(session.pickPoint({0, 0}));
	EXPECT_FALSE(session.selectRoute(0));
	auto pauseCompleted = paused.get_future();
	ASSERT_TRUE(session.requestPause([&paused] { paused.set_value(); }));
	EXPECT_EQ(session.state(), State::Pausing);
	EXPECT_FALSE(session.resume());
	EXPECT_EQ(pauseCompleted.wait_for(25ms), std::future_status::timeout);
	releaseFrame.set_value();
	ASSERT_EQ(pauseCompleted.wait_for(2s), std::future_status::ready);
	session.finishControl();
	EXPECT_EQ(session.state(), State::Paused);
	EXPECT_FALSE(session.resetRoute());
	auto stopCompleted = stopped.get_future();
	ASSERT_TRUE(session.requestStop([&stopped] { stopped.set_value(); }));
	EXPECT_EQ(session.state(), State::Stopping);
	ASSERT_EQ(stopCompleted.wait_for(2s), std::future_status::ready);
	session.finishControl();
	EXPECT_EQ(session.state(), State::Stopped);
	EXPECT_FALSE(session.beginSetting());
	ASSERT_TRUE(session.resetRoute());
	EXPECT_TRUE(session.options().empty());
	EXPECT_EQ(transitions,
			  (std::vector<State>{State::SettingRoute,
								  State::Calculating,
								  State::ChoosingRoute,
								  State::Starting,
								  State::Running,
								  State::Pausing,
								  State::Paused,
								  State::Stopping,
								  State::Stopped,
								  State::NotStarted}));
	session.shutdown();
	EXPECT_EQ(session.state(), State::Stopped);
	EXPECT_FALSE(session.resume());
}

// Verifies pause preserves packets and resume drains them before acquisition.
TEST(ProtectedQueueTest, PreservesPacketsAndDrainsBeforeNewInput)
{
	nav::session::ProtectedQueue queue(3);
	queue.resume();
	ASSERT_TRUE(queue.waitToProduce({}));
	queue.push({{1, {}, {}}, "first", false});
	ASSERT_TRUE(queue.waitToProduce({}));
	queue.push({{2, {}, {}}, "second", false});
	queue.pause();
	EXPECT_EQ(queue.pending(), 2);
	queue.resume();

	std::promise<bool> producerReady;
	auto produced = producerReady.get_future();
	std::jthread producer([&](std::stop_token stop) {
		const bool allowed = queue.waitToProduce(stop);
		if (allowed)
			queue.push({{3, {}, {}}, "third", false});
		producerReady.set_value(allowed);
	});
	auto first = queue.waitAndPop({});
	ASSERT_TRUE(first);
	EXPECT_EQ(first->sentence, "first");
	queue.complete();
	auto second = queue.waitAndPop({});
	ASSERT_TRUE(second);
	EXPECT_EQ(second->sentence, "second");
	EXPECT_EQ(produced.wait_for(25ms), std::future_status::timeout);
	queue.complete();
	ASSERT_EQ(produced.wait_for(2s), std::future_status::ready);
	EXPECT_TRUE(produced.get());
	EXPECT_EQ(queue.pending(), 1);
	queue.pause();
	queue.stop();
	EXPECT_EQ(queue.pending(), 0);
	EXPECT_FALSE(queue.waitAndPop({}));
	EXPECT_FALSE(queue.waitToProduce({}));
}
