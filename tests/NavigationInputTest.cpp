#include "nav/Routing/LocalProjection.h"
#include "nav/Gnss/NmeaRmc.h"
#include "nav/Routing/RoutePlanner.h"

#include <gtest/gtest.h>

using namespace std::chrono_literals;

// Verifies RMC coordinates, UTC date, and checksum rejection.
TEST(NmeaRmcTest, DecodesCoordinatesAndUtcDateFromReceiverSentence)
{
	const auto fix = nav::gnss::parseRmc("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
	ASSERT_TRUE(fix);
	EXPECT_NEAR(fix->degrees.y, 48.1173, 1e-9);
	EXPECT_NEAR(fix->degrees.x, 11.5166666667, 1e-9);
	const auto expected = std::chrono::sys_days{std::chrono::year{1994} / 3 / 23} + 12h + 35min + 19s;
	EXPECT_TRUE(fix->measuredAt == expected);
	EXPECT_FALSE(nav::gnss::parseRmc("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00"));
}

// Verifies that RMC round-tripping preserves time and local position.
TEST(NmeaRmcTest, PreservesTimeAndMapPositionThroughSimulatedReceiver)
{
	const nav::routing::LocalProjection projection({2.165, 41.39});
	const auto time = std::chrono::sys_days{std::chrono::year{2026} / 11 / 23} + 123ms;
	const auto fix = nav::gnss::parseRmc(nav::gnss::encodeRmc({projection.unproject({-100, 200}), time}));
	ASSERT_TRUE(fix);
	const auto local = projection.project(fix->degrees);
	EXPECT_NEAR(local.x, -100, 0.01);
	EXPECT_NEAR(local.y, 200, 0.01);
	EXPECT_TRUE(fix->measuredAt == time);
}

// Verifies shortest-path selection honors one-way roads.
TEST(RoadGraphTest, PicksTheShortestLegalPathAndRespectsOneWay)
{
	nav::routing::StreetMap map;
	map.streets = {
		{{1, 2, 3}, {{0, 0}, {10, 0}, {20, 0}}, true},
		{{3, 4, 1}, {{20, 0}, {10, 20}, {0, 0}}, false},
		{{5, 6}, {{30, 30}, {40, 30}}, false},
	};
	const nav::routing::RoadGraph graph(map);
	const nav::routing::RoutePlanner planner(graph);
	const auto outbound = planner.shortestRoute(1, 3);
	ASSERT_EQ(outbound.size(), 3);
	EXPECT_DOUBLE_EQ(outbound[1].y, 0);
	const auto inbound = planner.shortestRoute(3, 1);
	ASSERT_EQ(inbound.size(), 3);
	EXPECT_DOUBLE_EQ(inbound[1].y, 20);
	EXPECT_TRUE(planner.shortestRoute(1, 5).empty());
}
