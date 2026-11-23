#include "nav/Routing/OpenStreetMap.h"

#include "nav/Routing/LocalProjection.h"
#include "nav/Logger/Log.h"
#include "nav/Geometry/RouteGeometry.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <optional>
#include <pugixml.hpp>
#include <string>
#include <string_view>
#include <unordered_map>

namespace nav::routing
{
	namespace
	{
		constexpr double MaxLongitudeDegrees = 180;
		constexpr double MaxLatitudeDegrees = 90;

		struct Way
		{
			std::vector<std::int64_t> nodes;    // Ordered node references from the OSM way.
			std::string highway;                // OSM highway classification used to identify drivable ways.
			std::string oneway;                 // OSM travel-direction tag used to orient road edges.
			std::string junction;               // Junction tag used to recognize one-way roundabouts.
			std::string access;                 // General access restriction used as a routing fallback.
			std::string vehicle;                // Vehicle access override applied before the general restriction.
			std::string motorVehicle;           // Motor vehicle access override used for driving routes.
			std::string motorcar;               // Passenger-car access override with the highest priority.
			std::string building;               // Building tag used to identify footprint polygons.
		};

		// Reads a numeric XML attribute and reports malformed map values.
		template <typename Number> std::optional<Number> readNumber(const pugi::xml_node& node, const char* name)
		{
			const std::string_view text = node.attribute(name).value();
			Number value{};
			const auto end = text.data() + text.size();
			const auto [next, error] = std::from_chars(text.data(), end, value);
			if (error != std::errc{} || next != end)
			{
				nav::logger::error(std::string("Invalid OSM attribute: ") + name);
				return std::nullopt;
			}
			return value;
		}

		// Collects node references and tags used to classify an OSM way.
		std::optional<Way> readWay(const pugi::xml_node& element)
		{
			Way way;
			for (const auto& node : element.children("nd"))
			{
				const auto ref = readNumber<std::int64_t>(node, "ref");
				if (!ref)
					return std::nullopt;
				way.nodes.push_back(*ref);
			}
			for (const auto& tag : element.children("tag"))
			{
				const std::string_view key = tag.attribute("k").value();
				const auto value = tag.attribute("v").value();
				if (key == "highway")
					way.highway = value;
				else if (key == "oneway")
					way.oneway = value;
				else if (key == "junction")
					way.junction = value;
				else if (key == "access")
					way.access = value;
				else if (key == "vehicle")
					way.vehicle = value;
				else if (key == "motor_vehicle")
					way.motorVehicle = value;
				else if (key == "motorcar")
					way.motorcar = value;
				else if (key == "building")
					way.building = value;
			}
			return way;
		}

		// Applies road and access tags to decide whether routing can use the way.
		bool isDrivable(const Way& way)
		{
			constexpr const char* Highways[] = {
				"motorway",
				"motorway_link",
				"trunk",
				"trunk_link",
				"primary",
				"primary_link",
				"secondary",
				"secondary_link",
				"tertiary",
				"tertiary_link",
				"unclassified",
				"residential",
				"living_street",
				"service",
				"road",
			};
			if (std::ranges::none_of(Highways, [&](const char* highway) { return way.highway == highway; }))
				return false;
			for (const auto* access : {&way.motorcar, &way.motorVehicle, &way.vehicle, &way.access})
			{
				if (!access->empty())
					return *access != "no" && *access != "private";
			}
			return true;
		}
	}

	// Parses OSM XML into local street geometry and building footprints for the viewer and router.
	std::optional<StreetMap> loadStreetMapFromOsmXml(std::istream& input)
	{
		pugi::xml_document document;
		const auto result = document.load(input);
		if (!result)
		{
			nav::logger::error(std::string("Invalid OpenStreetMap XML: ") + result.description());
			return std::nullopt;
		}
		const auto root = document.document_element();
		if (std::string_view(root.name()) != "osm")
		{
			nav::logger::error("The file must be an OpenStreetMap XML extract.");
			return std::nullopt;
		}

		std::unordered_map<std::int64_t, nav::geometry::Point> coordinates;
		for (const auto& node : root.children("node"))
		{
			const auto id = readNumber<std::int64_t>(node, "id");
			const auto longitude = readNumber<double>(node, "lon");
			const auto latitude = readNumber<double>(node, "lat");
			if (!id || !longitude || !latitude)
				return std::nullopt;
			if (!(std::abs(*longitude) <= MaxLongitudeDegrees && std::abs(*latitude) <= MaxLatitudeDegrees))
			{
				nav::logger::error("A map node has invalid coordinates.");
				return std::nullopt;
			}
			coordinates.emplace(*id, nav::geometry::Point{*longitude, *latitude});
		}

		std::vector<Way> ways;
		std::vector<std::vector<std::int64_t>> buildingNodes;
		for (const auto& element : root.children("way"))
		{
			auto way = readWay(element);
			if (!way)
				return std::nullopt;
			if (isDrivable(*way))
				ways.push_back(std::move(*way));
			else if (!way->building.empty() && way->building != "no" && way->nodes.size() >= 4 &&
					 way->nodes.front() == way->nodes.back())
				buildingNodes.push_back(std::move(way->nodes));
		}

		StreetMap map;
		std::optional<LocalProjection> projection;
		for (auto& way : ways)
		{
			if (way.oneway == "-1")
				std::ranges::reverse(way.nodes);
			const bool oneWay = way.oneway == "-1" || way.oneway == "yes" || way.oneway == "true" || way.oneway == "1" ||
								(way.oneway.empty() && (way.junction == "roundabout" || way.highway == "motorway"));
			Street street;
			street.oneWay = oneWay;
			double length = 0;
			// Commits a continuous drivable street section and resets the accumulator.
			auto finishStreet = [&] {
				if (length > 0)
					map.streets.push_back(std::move(street));
				street = Street{};
				street.oneWay = oneWay;
				length = 0;
			};
			for (auto id : way.nodes)
			{
				const auto node = coordinates.find(id);
				if (node == coordinates.end())
				{
					finishStreet();
					continue;
				}
				if (!projection)
				{
					map.originDegrees = node->second;
					projection.emplace(map.originDegrees);
				}
				const auto point = projection->project(node->second);
				if (!street.points.empty())
					length += nav::geometry::distance(street.points.back(), point);
				street.points.push_back(point);
				street.nodeIds.push_back(id);
			}
			finishStreet();
		}
		if (map.streets.empty())
		{
			nav::logger::error("No drivable street geometry was found in this extract.");
			return std::nullopt;
		}
		for (const auto& nodes : buildingNodes)
		{
			std::vector<nav::geometry::Point> footprint;
			for (auto id : nodes)
			{
				const auto node = coordinates.find(id);
				if (node == coordinates.end())
					break;
				footprint.push_back(projection->project(node->second));
			}
			if (footprint.size() == nodes.size())
				map.buildings.push_back(std::move(footprint));
		}
		return map;
	}
}    // namespace nav::routing
