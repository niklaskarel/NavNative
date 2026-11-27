#include "nav/Tracking/RouteTracker.h"

#include <gtest/gtest.h>

#include <limits>

namespace
{
	using namespace std::chrono_literals;

	class RouteTrackerTest : public testing::Test
	{
	protected:
		nav::tracking::RouteTracker m_route{
			{{0, 0}, {100, 0}, {100, 100}}};    // Right-angle route fixture for projection and progress assertions.
	};

	// Verifies nearest-segment projection, progress, and remaining distance.
	TEST_F(RouteTrackerTest, ProjectsOntoSegment)
	{
		const auto match = m_route.update({40, 10});
		ASSERT_TRUE(match);
		EXPECT_DOUBLE_EQ(match->position.x, 40);
		EXPECT_DOUBLE_EQ(match->position.y, 0);
		EXPECT_DOUBLE_EQ(match->progressMeters, 40);
		EXPECT_DOUBLE_EQ(match->remainingMeters, 160);
	}

	// Verifies that matching includes distance traveled before a turn.
	TEST_F(RouteTrackerTest, CountsProgressAfterTurn)
	{
		EXPECT_DOUBLE_EQ(m_route.update({110, 80})->progressMeters, 180);
	}

	// Verifies interpolation uses the correct segment after a turn.
	TEST_F(RouteTrackerTest, InterpolatesPositionAfterTurn)
	{
		const auto position = m_route.pointAt(140);
		EXPECT_DOUBLE_EQ(position.x, 100);
		EXPECT_DOUBLE_EQ(position.y, 40);
	}

	// Verifies matching clamps to the route start.
	TEST_F(RouteTrackerTest, ClampsBeforeStart)
	{
		EXPECT_DOUBLE_EQ(m_route.update({-10, 0})->progressMeters, 0);
	}

	// Verifies matching clamps to the route end.
	TEST_F(RouteTrackerTest, ClampsBeyondEnd)
	{
		EXPECT_DOUBLE_EQ(m_route.update({100, 150})->progressMeters, 200);
	}

	// Verifies the cutoff for classifying an observation as off-route.
	TEST_F(RouteTrackerTest, FlagsObservationAwayFromRoute)
	{
		EXPECT_TRUE(m_route.update({50, -30})->offRoute);
		EXPECT_FALSE(m_route.update({50, -25})->offRoute);
	}

	// Verifies continuity keeps an observation on the current parallel segment.
	TEST(RouteTrackerContinuityTest, RejectsJumpToNearbyParallelSegment)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {300, 0}, {300, 25}, {0, 25}});
		const std::chrono::steady_clock::time_point start{};
		EXPECT_DOUBLE_EQ(tracker.track({{100, 0}, start})->progressMeters, 100);
		EXPECT_GT(tracker.update({102, 15})->progressMeters, 500);
		const auto match = tracker.track({{102, 15}, start + 100ms});
		ASSERT_TRUE(match);
		EXPECT_DOUBLE_EQ(match->progressMeters, 102);
		EXPECT_EQ(match->segment, 0);
	}

	// Verifies the progress window constrains forward movement and permits reversal.
	TEST(RouteTrackerContinuityTest, LimitsProgressChangesAndAllowsReversal)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {300, 0}});
		const std::chrono::steady_clock::time_point start{};
		tracker.track({{100, 0}, start});
		EXPECT_DOUBLE_EQ(tracker.track({{130, 0}, start + 100ms})->progressMeters, 113.5);
		EXPECT_DOUBLE_EQ(tracker.track({{108, 0}, start + 200ms})->progressMeters, 108);
	}

	// Verifies that the prediction follows a bend, expires, and gives way to GPS when it returns.
	TEST(RouteTrackerDeadReckoningTest, PredictsThroughBendAndExpires)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {125, 0}, {125, 100}});
		const std::chrono::steady_clock::time_point start{};
		EXPECT_FALSE(tracker.predictProgress(start, 10));
		ASSERT_TRUE(tracker.track({{90, 0}, start}));
		ASSERT_TRUE(tracker.track({{100, 0}, start + 1s}));

		const auto progress = tracker.predictProgress(start + 6s, 10);
		ASSERT_TRUE(progress);
		EXPECT_DOUBLE_EQ(*progress, 150);
		EXPECT_DOUBLE_EQ(tracker.pointAt(*progress).x, 125);
		EXPECT_DOUBLE_EQ(tracker.pointAt(*progress).y, 25);
		EXPECT_FALSE(tracker.predictProgress(start + 6001ms, 10));

		const auto recovered = tracker.track({{125, 20}, start + 6s});
		ASSERT_TRUE(recovered);
		EXPECT_DOUBLE_EQ(recovered->progressMeters, 145);
		const auto nextPrediction = tracker.predictProgress(start + 6100ms, 10);
		ASSERT_TRUE(nextPrediction);
		EXPECT_DOUBLE_EQ(*nextPrediction, 146);
	}

	// Verifies that supplied speed controls direction and changes only affect future motion.
	TEST(RouteTrackerDeadReckoningTest, IntegratesSpeedChangesDespiteBackwardGpsJitter)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {300, 0}});
		const std::chrono::steady_clock::time_point start{};
		ASSERT_TRUE(tracker.track({{100, 0}, start}));
		ASSERT_TRUE(tracker.track({{90, 0}, start + 1s}));
		const auto forward = tracker.predictProgress(start + 2s, 10);
		ASSERT_TRUE(forward);
		EXPECT_DOUBLE_EQ(*forward, 100);
		const auto stopped = tracker.predictProgress(start + 3s, 0);
		ASSERT_TRUE(stopped);
		EXPECT_DOUBLE_EQ(*stopped, 100);
		const auto reversed = tracker.predictProgress(start + 4s, -5);
		ASSERT_TRUE(reversed);
		EXPECT_DOUBLE_EQ(*reversed, 95);
		tracker.resetTracking();
		EXPECT_FALSE(tracker.predictProgress(start + 4s, -5));
	}

	// Verifies a signal gap or reset allows full-route matching again.
	TEST(RouteTrackerContinuityTest, ReacquiresAfterGapAndClearsStateOnReset)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {300, 0}, {300, 25}, {0, 25}});
		const std::chrono::steady_clock::time_point start{};
		tracker.track({{100, 0}, start});
		EXPECT_TRUE(tracker.track({{250, 25}, start + 100ms})->offRoute);
		EXPECT_EQ(tracker.track({{100, 25}, start + 2s})->segment, 2);
		tracker.resetTracking();
		EXPECT_EQ(tracker.track({{100, 0}, start})->segment, 0);
	}

	// Verifies the preceding segment remains eligible at the exact start of the progress window.
	TEST(RouteTrackerContinuityTest, KeepsSharedVertexAtWindowStart)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {100, 0}, {200, 0}});
		const std::chrono::steady_clock::time_point start{};
		ASSERT_TRUE(tracker.track({{110, 0}, start}));

		const auto match = tracker.track({{100, 0}, start});
		ASSERT_TRUE(match);
		EXPECT_DOUBLE_EQ(match->progressMeters, 100);
		EXPECT_EQ(match->segment, 0);
	}

	// Verifies an older fix is discarded.
	TEST(RouteTrackerContinuityTest, RejectsOutOfOrderFixes)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {300, 0}});
		const std::chrono::steady_clock::time_point start{};
		tracker.track({{100, 0}, start + 100ms});
		EXPECT_FALSE(tracker.track({{100, 0}, start}));
	}

	// Verifies a non-finite fix does not block the next valid fix.
	TEST(RouteTrackerContinuityTest, DiscardsNonFiniteFixAndContinuesTracking)
	{
		nav::tracking::RouteTracker tracker({{0, 0}, {300, 0}});
		const std::chrono::steady_clock::time_point start{};
		EXPECT_FALSE(tracker.track({{std::numeric_limits<double>::infinity(), 0}, start + 1s}));
		const auto match = tracker.track({{100, 0}, start});
		ASSERT_TRUE(match);
		EXPECT_DOUBLE_EQ(match->progressMeters, 100);
	}

	// Verifies duplicate route points do not disrupt matching.
	TEST(RouteTrackerConstructionTest, SkipsDuplicateVertices)
	{
		const nav::tracking::RouteTracker route({{0, 0}, {0, 0}, {100, 0}});
		EXPECT_DOUBLE_EQ(route.update({25, 2})->progressMeters, 25);
	}

	// Verifies a zero-length route cannot produce a match.
	TEST(RouteTrackerConstructionTest, RejectsDegenerateRoute)
	{
		const nav::tracking::RouteTracker tracker({{0, 0}, {0, 0}});
		EXPECT_FALSE(tracker.update({0, 0}));
	}
}    // namespace
