#pragma once

#include "nav/Routing/OpenStreetMap.h"
#include "nav/Tracking/RouteTracker.h"

#include <QPainterPath>
#include <QWidget>

#include <array>
#include <functional>
#include <optional>
#include <vector>

namespace nav::viewer
{
	/// Displays the planned route, recent GPS fixes, and current tracking frame.
	class MapCanvas : public QWidget
	{
	public:
		/// Stores the map and calculates bounds used to fit it in the widget.
		explicit MapCanvas(nav::routing::StreetMap map);

		/// Updates the displayed positions and GPS trail when a matched frame arrives.
		void setFrame(nav::geometry::Point truth, nav::geometry::Point gps, const nav::tracking::Match& match);
		/// Clears the GPS trail when resetting the displayed drive.
		void clearObservations();
		/// Updates the active and alternate paths used by the preview and map selection.
		void setRoutes(nav::geometry::SharedRoute first,
					   nav::geometry::SharedRoute second,
					   std::size_t selected,
					   bool showAlternates);
		/// Updates markers for the ordered stops and shows the destination after confirmation.
		void setWaypoints(const std::vector<nav::geometry::Point>& positions, bool confirmed);
		/// Finds the closest displayed route within the map click tolerance.
		std::optional<std::size_t> routeAt(nav::geometry::Point point) const;
		/// Removes vehicle markers and GPS history before a new drive.
		void clearFrame();
		/// Registers the viewer callback that handles a map click in local coordinates.
		void setPickHandler(std::function<void(nav::geometry::Point)> handler);
		/// Registers the callback for right-clicks on endpoint markers.
		void setClearEndpointsHandler(std::function<bool()> handler);

	protected:
		/// Draws the streets, buildings, routes, and latest vehicle positions.
		void paintEvent(QPaintEvent* event) override;
		/// Converts a map click to local meters and forwards it to the viewer.
		void mousePressEvent(QMouseEvent* event) override;

	private:
		/// Calculates the pixels per meter needed to fit the map in the widget.
		double mapScale() const;
		/// Computes the screen translation that centers the map at the given scale.
		QPointF mapOrigin(double scale) const;
		/// Rebuilds one-way arrows for the current viewport.
		void rebuildDirectionOverlay(double scale, QPointF origin);

		QPainterPath m_directionOverlay;    // Cached one-way chevrons in screen coordinates.
		QSize m_directionViewport;          // Widget dimensions used to build the overlay.

		nav::routing::StreetMap m_streetMap;                        // Road and building geometry rendered by the map widget.
		QRectF m_mapBounds;                                         // World-coordinate bounds used to scale and center the map.
		bool m_hasFrame = false;                                    // Controls whether the latest vehicle markers are drawn.
		nav::geometry::Point m_truth{};                             // Latest ground-truth position drawn in green.
		nav::geometry::Point m_gps{};                               // Latest simulated GPS position drawn in yellow.
		nav::tracking::Match m_match{};                             // Latest tracker estimate drawn in blue.
		std::vector<nav::geometry::Point> m_observations;           // Recent GPS readings used to draw the observation trail.
		std::vector<nav::geometry::Point> m_waypointMarkers;      // Snapped stops in click order.
		bool m_waypointsConfirmed = false;                         // Applies the destination style to the last stop.
		std::array<nav::geometry::SharedRoute, 2> m_routes;    // Shared candidate route geometry for drawing and hit testing.
		std::size_t m_selected = 0;                            // Index of the route drawn with the selected style.
		bool m_showAlternates = false;                         // Controls whether a second candidate route is displayed.
		std::function<void(nav::geometry::Point)> m_pickHandler;    // Receives clicked map coordinates in local meters.
		std::function<bool()> m_clearEndpoints;                     // Requests clearing route setup after a marker is clicked.
	};
}    // namespace nav::viewer
