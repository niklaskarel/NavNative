#include "nav/Routing/RoutePlanner.h"

#include "nav/Geometry/RouteGeometry.h"

#include <queue>

#include <algorithm>
#include <tuple>

namespace nav::routing
{
	// Binds route searches to the session's immutable road graph.
	RoutePlanner::RoutePlanner(const RoadGraph& graph)
		: m_graph(graph)
	{
	}

	// Converts graph node IDs into the coordinates used to display and drive a route.
	nav::geometry::Route RoutePlanner::geometry(const std::vector<NodeId>& nodes) const
	{
		nav::geometry::Route route;
		route.reserve(nodes.size());
		for (auto node : nodes)
		{
			const auto next = m_graph.position(node);
			if (route.empty() || route.back().x != next.x || route.back().y != next.y)
				route.push_back(next);
		}
		return route;
	}

	// Returns the points on the shortest path between two graph nodes.
	std::vector<nav::geometry::Point> RoutePlanner::shortestRoute(NodeId start, NodeId destination) const
	{
		return geometry(search(start, destination).nodes);
	}

	// Finds the best route and the shortest distinct alternative for the viewer.
	std::vector<RouteOption> RoutePlanner::routeOptions(NodeId start, NodeId destination, std::stop_token stop) const
	{
		auto best = search(start, destination, {}, {}, stop);
		auto bestGeometry = geometry(best.nodes);
		if (bestGeometry.size() < 2 || stop.stop_requested())
			return {};

		// Yen's second path: try peeling away from the shortest path at each node.
		std::unordered_set<NodeId> excluded;
		double prefixMeters = 0;
		std::optional<RouteOption> alternate;
		for (std::size_t i = 0; i + 1 < best.nodes.size() && !stop.stop_requested(); ++i)
		{
			const auto from = best.nodes[i];
			const auto next = best.nodes[i + 1];
			auto spur = search(from, destination, excluded, std::pair{from, next}, stop);
			const double meters = prefixMeters + spur.meters;    // Accumulated path cost used to compare route alternatives.
			if (!spur.nodes.empty() && (!alternate || meters < alternate->meters))
			{
				std::vector<NodeId> full(best.nodes.begin(), best.nodes.begin() + i);
				full.insert(full.end(), spur.nodes.begin(), spur.nodes.end());
				auto points = geometry(full);
				const bool sameGeometry =
					std::ranges::equal(points, bestGeometry, [](nav::geometry::Point a, nav::geometry::Point b) {
						return a.x == b.x && a.y == b.y;
					});
				if (points.size() >= 2 && !sameGeometry)
					alternate = RouteOption{std::make_shared<const nav::geometry::Route>(std::move(points)), meters};
			}
			excluded.insert(from);
			prefixMeters += nav::geometry::distance(m_graph.position(from), m_graph.position(next));
		}

		if (stop.stop_requested())
			return {};
		std::vector<RouteOption> results;
		results.push_back({std::make_shared<const nav::geometry::Route>(std::move(bestGeometry)), best.meters});
		if (alternate)
			results.push_back(std::move(*alternate));
		return results;
	}

	// Runs A* while honoring excluded nodes, an excluded edge, and cancellation.
	RoutePlanner::Path RoutePlanner::search(NodeId start,
											NodeId destination,
											const std::unordered_set<NodeId>& excluded,
											std::optional<std::pair<NodeId, NodeId>> excludedEdge,
											std::stop_token stop) const
	{
		struct Candidate
		{
			NodeId id;          // Graph node represented by this queued A* candidate.
			double used;        // Distance already traveled from the start to this candidate.
			double estimate;    // Estimated total route distance used to prioritize candidates.

			// Orders A* candidates by estimated cost and then node ID.
			bool operator>(const Candidate& other) const
			{
				return std::tie(estimate, id) > std::tie(other.estimate, other.id);
			}
		};

		std::priority_queue<Candidate, std::vector<Candidate>, std::greater<>> open;
		std::unordered_map<NodeId, double> costs{{start, 0}};
		std::unordered_map<NodeId, NodeId> previous;
		const auto finish = m_graph.position(destination);
		open.push({start, 0, nav::geometry::distance(m_graph.position(start), finish)});
		while (!open.empty() && !stop.stop_requested())
		{
			const auto current = open.top();
			open.pop();
			if (current.used > costs.at(current.id))
				continue;
			if (current.id == destination)
			{
				Path path;
				path.meters = current.used;
				for (auto node = destination;; node = previous.at(node))
				{
					path.nodes.push_back(node);
					if (node == start)
						break;
				}
				std::ranges::reverse(path.nodes);
				return path;
			}
			for (const auto& edge : m_graph.edges(current.id))
			{
				if (excluded.contains(edge.destination) || excludedEdge == std::pair{current.id, edge.destination})
					continue;
				const double next = current.used + edge.meters;
				const auto old = costs.find(edge.destination);
				if (old != costs.end() && next >= old->second)
					continue;
				costs[edge.destination] = next;
				previous[edge.destination] = current.id;
				// Straight-line distance is a lower bound on distance through the street graph.
				open.push({edge.destination, next, next + nav::geometry::distance(m_graph.position(edge.destination), finish)});
			}
		}
		return {};
	}
}    // namespace nav::routing
