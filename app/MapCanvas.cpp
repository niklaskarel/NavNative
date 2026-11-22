#include "MapCanvas.h"

#include "CityMap.h"

#include <QPainter>
#include <QPainterPath>

#include <algorithm>

MapCanvas::MapCanvas()
{
	setMinimumSize(640, 480);
}

void MapCanvas::setFrame(nav::Point truth, nav::Point gps, const nav::Match& match)
{
	m_truth = truth;
	m_gps = gps;
	m_match = match;
	m_observations.push_back(gps);
	if (m_observations.size() > 300)
	{
		m_observations.erase(m_observations.begin());
	}
	update();
}

void MapCanvas::clearObservations()
{
	m_observations.clear();
}

void MapCanvas::paintEvent(QPaintEvent*)
{
	using namespace city;

	QPainter p(this);
	p.setRenderHint(QPainter::Antialiasing);
	p.fillRect(rect(), QColor("#111827"));
	const double scale = std::min((width() - 64.0) / (MapWidth + 20), (height() - 72.0) / (MapHeight + 20));
	const QPointF origin((width() - MapWidth * scale) / 2, (height() + MapHeight * scale) / 2 + 12);
	// World y increases north, while Qt screen y increases downward.
	const auto screen = [&](nav::Point point) { return origin + QPointF(point.x * scale, -point.y * scale); };
	m_cityRenderer.draw(p, origin, scale);

	QPainterPath path;
	path.moveTo(screen(city::route().front()));
	for (const auto point : city::route())
	{
		path.lineTo(screen(point));
	}
	p.setBrush(Qt::NoBrush);
	p.setPen(QPen(QColor("#81a1c1"), 2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	p.drawPath(path);
	p.setPen(QPen(QColor("#fbbf24"), 2));
	for (auto point : m_observations)
		p.drawEllipse(screen(point), 1.5, 1.5);
	p.setPen(QPen(QColor("#f87171"), 1));
	p.drawLine(screen(m_gps), screen(m_match.position));
	auto dot = [&](nav::Point point, QColor color) {
		p.setBrush(color);
		p.setPen(Qt::NoPen);
		p.drawEllipse(screen(point), 6, 6);
	};
	dot(m_truth, QColor("#34d399"));
	dot(m_gps, QColor("#fbbf24"));
	dot(m_match.position, QColor("#60a5fa"));
	p.setPen(Qt::white);
	p.drawText(15, 25, "Eixample-inspired grid    |    Green: true position    Yellow: GPS    Blue: route match");
}
