#pragma once

#include "nav/Routing/RoadGraph.h"

#include <optional>
#include <span>
#include <stop_token>
#include <unordered_set>

namespace nav::routing
{
	/// Finds the two shortest distinct routes through an immutable directed street graph.
	class RoutePlanner
	{
	public:
		/// Binds route searches to the session's immutable road graph.
		explicit RoutePlanner(const RoadGraph& graph);
		/// Returns the points on the shortest path between two graph nodes.
		nav::geometry::Route shortestRoute(nav::routing::NodeId start, nav::routing::NodeId destination) const;
		/// Finds the best route and the shortest distinct alternative for the viewer.
		std::vector<RouteOption>
			routeOptions(nav::routing::NodeId start, nav::routing::NodeId destination, std::stop_token stop = {}) const;

		/// Finds up to two routes that visit all selected stops in order.
		std::vector<RouteOption> routeOptions(std::span<const NodeId> stops, std::stop_token stop = {}) const;

	private:
		struct Path
		{
			std::vector<NodeId> nodes;    // Ordered graph nodes reconstructed by the path search.
			double meters = 0;            // Accumulated path cost used to compare route alternatives.
		};

		/// Runs A* while honoring excluded nodes, an excluded edge, and cancellation.
		Path search(nav::routing::NodeId start,
					nav::routing::NodeId destination,
					const std::unordered_set<NodeId>& excluded = {},
					std::optional<std::pair<NodeId, NodeId>> excludedEdge = {},
					std::stop_token stop = {}) const;
		/// Converts graph node IDs into the coordinates used to display and drive a route.
		nav::geometry::Route geometry(const std::vector<NodeId>& nodes) const;

		const RoadGraph& m_graph;    // Immutable graph borrowed for A* searches.
	};
}    // namespace nav::routing
