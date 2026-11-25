#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace nav
{
	namespace geometry
	{
		/// A point in a local Cartesian coordinate system measured in meters.
		struct Point
		{
			double x = 0;    // Horizontal coordinate, measured in local meters for projected positions.
			double y = 0;    // Vertical coordinate, measured in local meters for projected positions.
		};

		using Route = std::vector<Point>;
		using SharedRoute = std::shared_ptr<const Route>;

		/// The closest point and its distance along a segment from the start.
		struct SegmentProjection
		{
			Point position;              // Local point returned by segment projection or carried in a fix or match.
			double distanceFromStart;    // Meters from the segment start to the projected position.
		};
	}    // namespace geometry

	namespace tracking
	{
		/// The nearest route position and distances measured along and away from the route.
		struct Match
		{
			geometry::Point position;              // Local point returned by segment projection or carried in a fix or match.
			double progressMeters = 0;             // Distance along the route from its start, measured in meters.
			double remainingMeters = 0;            // Distance from the matched position to the destination.
			double distanceFromRouteMeters = 0;    // Straight-line error between the GPS reading and its match.
			int segment = 0;                       // Index of the edge containing the matched position.
			bool offRoute = false;                 // Whether the GPS error exceeded the allowable distance from the route.
		};

		/// A GPS position and its monotonic acquisition time.
		struct GpsFix
		{
			geometry::Point position;    // Local point returned by segment projection or carried in a fix or match.
			std::chrono::steady_clock::time_point
				timestamp;    // Monotonic capture time used to enforce fix order and track continuity.
		};
	}    // namespace tracking

	namespace simulation
	{
		/// The simulator's ground truth and GPS reading for one update.
		struct DriveSample
		{
			double progressMeters;         // Distance along the route from its start, measured in meters.
			nav::geometry::Point truth;    // Simulated vehicle position before GPS noise and offset are applied.
			nav::geometry::Point gps;      // Simulated receiver reading in local meters.
		};

		/// One simulated receiver message and its accompanying ground truth.
		struct GnssPacket
		{
			DriveSample sample;      // Ground-truth and GPS metadata paired with the receiver sentence.
			std::string sentence;    // Encoded NMEA RMC line passed to the tracking worker.
			bool arrived;            // Whether this frame or packet reached the destination.
		};
	}    // namespace simulation

	namespace session
	{
		/// A simulated reading and its corresponding route match.
		struct DriveFrame : simulation::DriveSample
		{
			tracking::Match match;    // Route estimate paired with the simulated sample.
			bool arrived = false;     // Whether this frame or packet reached the destination.
		};
	}    // namespace session

	namespace routing
	{
		using NodeId = std::int64_t;
		/// A drivable way in local meters. Shared node IDs identify street connections.
		struct Street
		{
			std::vector<NodeId> nodeIds;                // OSM node identifiers used to connect shared street vertices.
			std::vector<nav::geometry::Point> points;    // Ordered geometry used by the router, tracker, and viewer.
			bool oneWay = false;                         // Whether routing may traverse the street only in input order.
		};

		/// A user-selected route endpoint(s) snapped to the road graph.
		struct Waypoint
		{
			std::vector<NodeId> nodeIds;
			std::vector < geometry::Point> positions;
		};

		struct StreetMap
		{
			std::vector<Street> streets;                                 // Drivable ways used to build the routing graph.
			std::vector<std::vector<nav::geometry::Point>> buildings;    // Building footprints drawn as map polygons.
			nav::geometry::Point originDegrees;                          // Longitude and latitude of the local projection origin.
		};

		struct RouteOption
		{
			nav::geometry::SharedRoute points;    // Ordered geometry used by the router, tracker, and viewer.
			double meters;                        // Length of the candidate route in meters.
		};
	}    // namespace routing

	namespace gnss
	{
		/// Geographic coordinates and UTC measurement time decoded from an RMC sentence.
		struct RmcFix
		{
			geometry::Point degrees;    // Decoded position in longitude and latitude degrees.
			std::chrono::sys_time<std::chrono::milliseconds>
				measuredAt;    // UTC measurement time reported by the receiver sentence.
		};
	}    // namespace gnss
}    // namespace nav
