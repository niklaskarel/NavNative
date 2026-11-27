#include "MapCanvas.h"

#include "nav/Geometry/RouteGeometry.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav::viewer
{
	namespace
	{
		constexpr int MinimumMapWidth = 640;
		constexpr int MinimumMapHeight = 480;
		constexpr std::size_t ObservationLimit = 300;
		constexpr double RouteHitRadius = 12;
		constexpr double HorizontalViewportPadding = 64.0;
		constexpr double VerticalViewportPadding = 72.0;
		constexpr double WorldPaddingMeters = 20;
		constexpr double MapVerticalOffset = 12;
		constexpr double MinimumArrowStreetLength = 16;
		constexpr double ArrowSpacing = 48;
		constexpr double ArrowHalfWidth = 2.5;
		constexpr double ArrowHalfLength = 2;
		constexpr double OutlineStreetWidth = 8;
		constexpr double StreetWidth = 5;
		constexpr double SelectedRouteWidth = 4;
		constexpr double AlternateRouteWidth = 3;
		constexpr double ArrowOutlineWidth = 2.8;
		constexpr double ArrowWidth = 1.2;
		constexpr double MarkerRadius = 6;
		constexpr int BuildingOutlineWidth = 1;
		constexpr int ObservationPenWidth = 2;
		constexpr int MatchLineWidth = 1;
		constexpr double ObservationRadius = 1.5;
		constexpr int LegendX = 15;
		constexpr int LegendY = 8;
	}

	// Stores the map and calculates bounds used to fit it in the widget.
	MapCanvas::MapCanvas(nav::routing::StreetMap map)
		: m_streetMap(std::move(map))
	{
		setMinimumSize(MinimumMapWidth, MinimumMapHeight);
		auto* overlayBar = new QWidget(this);
		auto* controls = new QHBoxLayout(overlayBar);
		controls->setContentsMargins(0, 0, 0, 0);
		const auto addOverlay = [&](const QString& text, const QString& color, Overlay overlay) {
			auto* checkbox = new QCheckBox(text, overlayBar);
			checkbox->setStyleSheet(QStringLiteral("color: %1;").arg(color));
			checkbox->setChecked(isOverlayVisible(overlay));
			connect(checkbox, &QCheckBox::toggled, this, [this, overlay](bool visible) {
				setOverlayVisible(overlay, visible);
			});
			controls->addWidget(checkbox);
		};
		addOverlay(QStringLiteral("● True position"), QStringLiteral("#34d399"), Overlay::TruePosition);
		addOverlay(QStringLiteral("● GPS position"), QStringLiteral("#fbbf24"), Overlay::GpsPosition);
		addOverlay(QStringLiteral("● Route match"), QStringLiteral("#60a5fa"), Overlay::RouteMatch);

		auto* arrowLegend = new QLabel(QStringLiteral("Arrows: one-way streets"), overlayBar);
		arrowLegend->setStyleSheet(QStringLiteral("color: white;"));
		controls->addWidget(arrowLegend);
		overlayBar->adjustSize();
		overlayBar->move(LegendX, LegendY);

		const double infinity = std::numeric_limits<double>::infinity();
		nav::geometry::Point low{infinity, infinity};
		nav::geometry::Point high{-infinity, -infinity};
		for (const auto& street : m_streetMap.streets)
		{
			for (const auto point : street.points)
			{
				low.x = std::min(low.x, point.x);
				low.y = std::min(low.y, point.y);
				high.x = std::max(high.x, point.x);
				high.y = std::max(high.y, point.y);
			}
		}
		m_mapBounds = QRectF(QPointF(low.x, low.y), QPointF(high.x, high.y));
	}

	void MapCanvas::setOverlayVisible(Overlay overlay, bool visible)
	{
		m_visibleOverlays.set(static_cast<std::size_t>(overlay), visible);
		update();
	}

	bool MapCanvas::isOverlayVisible(Overlay overlay) const
	{
		return m_visibleOverlays.test(static_cast<std::size_t>(overlay));
	}

	// Updates the displayed positions and GPS trail when a matched frame arrives.
	void MapCanvas::setFrame(const nav::session::DriveFrame& frame)
	{
		m_truth = frame.truth;
		m_gps = frame.gps;
		m_estimate.reset();
		m_isPrediction = frame.prediction.has_value();
		if (frame.match && !frame.match->offRoute)
			m_lastGpsProgress = frame.match->progressMeters;
		m_predictedProgress = frame.prediction ? std::optional{frame.prediction->progressMeters} : std::nullopt;
		if (frame.prediction)
			m_estimate = frame.prediction->position;
		else if (frame.match)
			m_estimate = frame.match->position;
		m_hasFrame = true;
		if (m_gps)
			m_observations.push_back(*m_gps);
		if (m_observations.size() > ObservationLimit)
		{
			m_observations.erase(m_observations.begin());
		}
		update();
	}

	// Clears the GPS trail when resetting the displayed drive.
	void MapCanvas::clearObservations()
	{
		m_observations.clear();
	}

	// Updates the active and alternate paths used by the preview and map selection.
	void MapCanvas::setRoutes(nav::geometry::SharedRoute first,
							  nav::geometry::SharedRoute second,
							  std::size_t selected,
							  bool showAlternates)
	{
		m_routes = {std::move(first), std::move(second)};
		m_selected = selected;
		m_showAlternates = showAlternates;
		update();
	}

	void MapCanvas::setWaypoints(const std::vector<nav::geometry::Point>& positions, bool confirmed)
	{
		m_waypointMarkers = positions;
		m_waypointsConfirmed = confirmed;
		update();
	}

	// Finds the closest displayed route within the map click tolerance.
	std::optional<std::size_t> MapCanvas::routeAt(nav::geometry::Point point) const
	{
		std::optional<std::size_t> hit;
		double bestDistance = RouteHitRadius / mapScale();
		for (std::size_t index = 0; index < m_routes.size(); ++index)
		{
			const auto& route = m_routes[index];
			if (!route || (!m_showAlternates && index != m_selected))
				continue;
			for (std::size_t i = 1; i < route->size(); ++i)
			{
				if (const auto projection = nav::geometry::projectOntoSegment(point, (*route)[i - 1], (*route)[i]))
				{
					const auto error = nav::geometry::distance(point, projection->position);
					if (error < bestDistance || (hit && error == bestDistance && index == m_selected))
					{
						bestDistance = error;
						hit = index;
					}
				}
			}
		}
		return hit;
	}

	// Removes vehicle markers and GPS history before a new drive.
	void MapCanvas::clearFrame()
	{
		m_hasFrame = false;
		m_lastGpsProgress.reset();
		m_predictedProgress.reset();
		clearObservations();
		update();
	}

	// Registers the viewer callback that handles a map click in local coordinates.
	void MapCanvas::setPickHandler(std::function<void(nav::geometry::Point)> handler)
	{
		m_pickHandler = std::move(handler);
	}

	// Sets the viewer callback used after a right-click hits an endpoint marker.
	void MapCanvas::setClearEndpointsHandler(std::function<bool()> handler)
	{
		m_clearEndpoints = std::move(handler);
	}

	// Calculates the pixels per meter needed to fit the map in the widget.
	double MapCanvas::mapScale() const
	{
		return std::min((width() - HorizontalViewportPadding) / (m_mapBounds.width() + WorldPaddingMeters),
						(height() - VerticalViewportPadding) / (m_mapBounds.height() + WorldPaddingMeters));
	}

	// Computes the screen translation that centers the map at the given scale.
	QPointF MapCanvas::mapOrigin(double scale) const
	{
		return {width() / 2.0 - m_mapBounds.center().x() * scale,
				height() / 2.0 + m_mapBounds.center().y() * scale + MapVerticalOffset};
	}

	// Converts a map click to local meters and forwards it to the viewer.
	void MapCanvas::mousePressEvent(QMouseEvent* event)
	{
		if (m_pickHandler && event->button() == Qt::LeftButton)
		{
			const double scale = mapScale();
			const auto local = (event->position() - mapOrigin(scale)) / scale;
			m_pickHandler({local.x(), -local.y()});
			event->accept();
			return;
		}
		if (event->button() == Qt::RightButton && m_clearEndpoints && m_clearEndpoints())
		{
			event->accept();
			return;
		}
		QWidget::mousePressEvent(event);
	}

	// Builds screen-space chevrons in the same vertex order used by the directed road graph.
	void MapCanvas::rebuildDirectionOverlay(double scale, QPointF origin)
	{
		m_directionViewport = size();
		m_directionOverlay = {};
		for (const auto& street : m_streetMap.streets)
		{
			if (!street.oneWay)
				continue;
			double totalLength = 0;
			for (std::size_t i = 1; i < street.points.size(); ++i)
				totalLength += nav::geometry::distance(street.points[i - 1], street.points[i]) * scale;
			if (totalLength < MinimumArrowStreetLength)
				continue;

			const auto count = std::max(1, static_cast<int>(totalLength / ArrowSpacing));
			const auto spacing = totalLength / count;
			double nextArrow = spacing / 2;
			double traveled = 0;
			for (std::size_t i = 1; i < street.points.size(); ++i)
			{
				const auto& from = street.points[i - 1];
				const auto& to = street.points[i];
				const QPointF delta((to.x - from.x) * scale, (from.y - to.y) * scale);
				const auto length = std::hypot(delta.x(), delta.y());
				if (length == 0)
					continue;

				const QPointF direction = delta / length;
				const QPointF side(-direction.y() * ArrowHalfWidth, direction.x() * ArrowHalfWidth);
				const QPointF start = origin + QPointF(from.x * scale, -from.y * scale);
				while (nextArrow <= traveled + length)
				{
					const auto center = start + direction * (nextArrow - traveled);
					const auto back = center - direction * ArrowHalfLength;
					m_directionOverlay.moveTo(back + side);
					m_directionOverlay.lineTo(center + direction * ArrowHalfLength);
					m_directionOverlay.lineTo(back - side);
					nextArrow += spacing;
				}
				traveled += length;
			}
		}
	}

	// Draws the streets, buildings, routes, and latest vehicle positions.
	void MapCanvas::paintEvent(QPaintEvent*)
	{
		QPainter p(this);
		p.setRenderHint(QPainter::Antialiasing);
		p.fillRect(rect(), QColor("#111827"));
		const double scale = mapScale();
		const auto origin = mapOrigin(scale);
		// World y increases north, while Qt screen y increases downward.
		const auto screen = [&](nav::geometry::Point point) { return origin + QPointF(point.x * scale, -point.y * scale); };
		p.setPen(QPen(QColor("#242c38"), BuildingOutlineWidth));
		p.setBrush(QColor("#49505c"));
		for (const auto& building : m_streetMap.buildings)
		{
			QPolygonF footprint;
			for (const auto point : building)
				footprint.push_back(screen(point));
			p.drawPolygon(footprint);
		}
		QPainterPath streets;
		for (const auto& street : m_streetMap.streets)
		{
			streets.moveTo(screen(street.points.front()));
			for (const auto point : street.points)
				streets.lineTo(screen(point));
		}
		p.strokePath(streets, QPen(QColor("#596474"), OutlineStreetWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		p.strokePath(streets, QPen(QColor("#303a49"), StreetWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

		// Draws one route using the selected or alternate style.
		const auto drawRoute = [&](std::size_t index) {
			const auto& route = m_routes[index];
			if (!route || route->empty())
				return;
			QPainterPath path;
			path.moveTo(screen(route->front()));
			for (std::size_t i = 1; i < route->size(); ++i)
				path.lineTo(screen((*route)[i]));
			const bool selected = index == m_selected;
			const QColor color(index == 0 ? "#75aaff" : "#ffb46a");
			p.strokePath(path,
						 QPen(color,
							  selected ? SelectedRouteWidth : AlternateRouteWidth,
							  selected ? Qt::SolidLine : Qt::DashLine,
							  Qt::RoundCap,
							  Qt::RoundJoin));
		};
		if (m_showAlternates)
			drawRoute(1 - m_selected);
		drawRoute(m_selected);

		const auto& route = m_routes[m_selected];
		if (isOverlayVisible(Overlay::RouteMatch) && route && m_lastGpsProgress && m_predictedProgress
			&& *m_predictedProgress > *m_lastGpsProgress)
		{
			QPainterPath predictedPath;
			double segmentStart = 0.0;
			for (std::size_t i = 1; i < route->size(); ++i)
			{
				const auto from = (*route)[i - 1];
				const auto to = (*route)[i];
				const double length = nav::geometry::distance(from, to);
				const double segmentEnd = segmentStart + length;
				const double begin = std::max(segmentStart, *m_lastGpsProgress);
				const double end = std::min(segmentEnd, *m_predictedProgress);
				if (begin < end)
				{
					const auto pointAtDistance = [&](double progress) {
						const double fraction = (progress - segmentStart) / length;
						return screen({std::lerp(from.x, to.x, fraction), std::lerp(from.y, to.y, fraction)});
					};
					if (predictedPath.isEmpty())
						predictedPath.moveTo(pointAtDistance(begin));
					predictedPath.lineTo(pointAtDistance(end));
				}
				if (segmentEnd >= *m_predictedProgress)
					break;
				segmentStart = segmentEnd;
			}
			p.strokePath(predictedPath,
						 QPen(QColor("#fb923c"), SelectedRouteWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		}

		if (m_directionViewport != size())
			rebuildDirectionOverlay(scale, origin);
		p.strokePath(m_directionOverlay,
					 QPen(QColor("#202938"), ArrowOutlineWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
		p.strokePath(m_directionOverlay, QPen(QColor("#f1e6c3"), ArrowWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

		p.setPen(Qt::NoPen);
		for (std::size_t i = 0; i < m_waypointMarkers.size(); ++i)
		{
			const bool destination = m_waypointsConfirmed && i + 1 == m_waypointMarkers.size();
			p.setBrush(QColor(destination ? "#d58df1" : (i == 0 ? "#e5e7eb" : "#7dd3fc")));
			p.drawEllipse(screen(m_waypointMarkers[i]), MarkerRadius, MarkerRadius);
		}
		const bool showGps = isOverlayVisible(Overlay::GpsPosition);
		const bool showMatch = isOverlayVisible(Overlay::RouteMatch);
		if (showGps)
		{
			p.setBrush(Qt::NoBrush);
			p.setPen(QPen(QColor("#fbbf24"), ObservationPenWidth));
			for (const auto point : m_observations)
				p.drawEllipse(screen(point), ObservationRadius, ObservationRadius);
		}
		if (m_hasFrame)
		{
			if (showGps && showMatch && m_gps && m_estimate && !m_isPrediction)
			{
				p.setPen(QPen(QColor("#f87171"), MatchLineWidth));
				p.drawLine(screen(*m_gps), screen(*m_estimate));
			}
			// Draws one colored position marker at its mapped screen coordinates.
			auto dot = [&](nav::geometry::Point point, QColor color) {
				p.setBrush(color);
				p.setPen(Qt::NoPen);
				p.drawEllipse(screen(point), MarkerRadius, MarkerRadius);
			};
			if (isOverlayVisible(Overlay::TruePosition))
				dot(m_truth, QColor("#34d399"));
			if (showGps && m_gps)
				dot(*m_gps, QColor("#fbbf24"));
			if (showMatch && m_estimate)
				dot(*m_estimate, QColor(m_isPrediction ? "#fb923c" : "#60a5fa"));
		}
	}
}    // namespace nav::viewer
