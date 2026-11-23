#include "nav/Routing/RoadGraph.h"

#include "nav/Geometry/RouteGeometry.h"

#include <algorithm>

namespace nav::routing
{
	// Builds directed edges from shared OSM nodes and street direction tags.
	RoadGraph::RoadGraph(const StreetMap& map)
	{
		for (const auto& street : map.streets)
		{
			if (street.nodeIds.empty())
				continue;
			m_nodes.try_emplace(street.nodeIds.front(), Node{street.points.front(), {}});
			for (std::size_t i = 1; i < street.nodeIds.size(); ++i)
			{
				const auto from = street.nodeIds[i - 1];
				const auto to = street.nodeIds[i];
				if (from == to)
					continue;
				auto& destination = m_nodes.try_emplace(to, Node{street.points[i], {}}).first->second;
				const auto meters = nav::geometry::distance(street.points[i - 1], street.points[i]);
				m_nodes.at(from).edges.push_back({to, meters});
				if (!street.oneWay)
					destination.edges.push_back({from, meters});
			}
		}
	}

	// Finds the graph node closest to a point picked in the viewer.
	RoadGraph::NodeId RoadGraph::nearestNode(nav::geometry::Point point) const
	{
		return std::min_element(m_nodes.begin(),
								m_nodes.end(),
								[&](const auto& a, const auto& b) {
									const auto first = nav::geometry::distance(point, a.second.position);
									const auto second = nav::geometry::distance(point, b.second.position);
									return first == second ? a.first < b.first : first < second;
								})
			->first;
	}

	// Returns a node's local coordinates for A* and route drawing.
	nav::geometry::Point RoadGraph::position(NodeId id) const
	{
		return m_nodes.at(id).position;
	}

	// Exposes the outgoing edges that the planner can traverse.
	std::span<const RoadGraph::Edge> RoadGraph::edges(NodeId id) const
	{
		return m_nodes.at(id).edges;
	}
}    // namespace nav::routing
