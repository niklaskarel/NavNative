#include "nav/RouteTracker.h"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{
	class RouteTrackerTest : public testing::Test
	{
	protected:
		nav::RouteTracker m_route{{{0, 0}, {100, 0}, {100, 100}}};
	};

	TEST_F(RouteTrackerTest, ProjectsOntoSegment)
	{
		const auto match = m_route.update({40, 10});
		EXPECT_DOUBLE_EQ(match.position.x, 40);
		EXPECT_DOUBLE_EQ(match.position.y, 0);
		EXPECT_DOUBLE_EQ(match.progressMeters, 40);
		EXPECT_DOUBLE_EQ(match.remainingMeters, 160);
	}

	TEST_F(RouteTrackerTest, CountsProgressAfterTurn)
	{
		EXPECT_DOUBLE_EQ(m_route.update({110, 80}).progressMeters, 180);
	}

	TEST_F(RouteTrackerTest, InterpolatesPositionAfterTurn)
	{
		const auto position = nav::pointAt(m_route, 140);
		EXPECT_DOUBLE_EQ(position.x, 100);
		EXPECT_DOUBLE_EQ(position.y, 40);
	}

	TEST_F(RouteTrackerTest, ClampsBeforeStart)
	{
		EXPECT_DOUBLE_EQ(m_route.update({-10, 0}).progressMeters, 0);
	}

	TEST_F(RouteTrackerTest, ClampsBeyondEnd)
	{
		EXPECT_DOUBLE_EQ(m_route.update({100, 150}).progressMeters, 200);
	}

	TEST_F(RouteTrackerTest, FlagsObservationAwayFromRoute)
	{
		EXPECT_TRUE(m_route.update({50, -30}).offRoute);
	}

	TEST(RouteTrackerConstructionTest, SkipsDuplicateVertices)
	{
		const nav::RouteTracker route({{0, 0}, {0, 0}, {100, 0}});
		EXPECT_DOUBLE_EQ(route.update({25, 2}).progressMeters, 25);
	}

	TEST(RouteTrackerConstructionTest, RejectsDegenerateRoute)
	{
		EXPECT_THROW((void)nav::RouteTracker({{0, 0}, {0, 0}}), std::invalid_argument);
	}
}    // namespace
