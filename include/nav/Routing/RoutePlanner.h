#pragma once

#include "nav/Routing/RoadGraph.h"

#include <optional>
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
		nav::geometry::Route shortestRoute(RoadGraph::NodeId start, RoadGraph::NodeId destination) const;
		/// Finds the best route and the shortest distinct alternative for the viewer.
		std::vector<RouteOption>
			routeOptions(RoadGraph::NodeId start, RoadGraph::NodeId destination, std::stop_token stop = {}) const;

	private:
		using NodeId = RoadGraph::NodeId;

		struct Path
		{
			std::vector<NodeId> nodes;    // Ordered graph nodes reconstructed by the path search.
			double meters = 0;            // Accumulated path cost used to compare route alternatives.
		};

		/// Runs A* while honoring excluded nodes, an excluded edge, and cancellation.
		Path search(NodeId start,
					NodeId destination,
					const std::unordered_set<NodeId>& excluded = {},
					std::optional<std::pair<NodeId, NodeId>> excludedEdge = {},
					std::stop_token stop = {}) const;
		/// Converts graph node IDs into the coordinates used to display and drive a route.
		nav::geometry::Route geometry(const std::vector<NodeId>& nodes) const;

		const RoadGraph& m_graph;    // Immutable graph borrowed for A* searches.
	};
}    // namespace nav::routing
