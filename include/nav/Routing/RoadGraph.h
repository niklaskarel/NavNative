#pragma once

#include "nav/Geometry/NavigationTypes.h"

#include <span>
#include <unordered_map>

namespace nav::routing
{
	/// Directed network of shared OSM nodes, with distance in meters as the edge cost.
	class RoadGraph
	{
	public:
		using NodeId = std::int64_t;
		/// Builds directed edges from shared OSM nodes and street direction tags.
		explicit RoadGraph(const StreetMap& map);
		/// Finds the graph node closest to a point picked in the viewer.
		NodeId nearestNode(nav::geometry::Point point) const;
		/// Returns a node's local coordinates for A* and route drawing.
		nav::geometry::Point position(NodeId id) const;

		struct Edge
		{
			NodeId destination;    // Node reached when the planner traverses this directed edge.
			double meters;         // Cost in meters of traversing the edge.
		};

		/// Exposes the outgoing edges that the planner can traverse.
		std::span<const Edge> edges(NodeId id) const;

	private:
		struct Node
		{
			nav::geometry::Point position;    // Local coordinates used by routing and click snapping.
			std::vector<Edge> edges;          // Outgoing directed edges considered during search.
		};

		std::unordered_map<NodeId, Node> m_nodes;    // Graph nodes and adjacency lists indexed by OSM node ID.
	};
}    // namespace nav::routing
