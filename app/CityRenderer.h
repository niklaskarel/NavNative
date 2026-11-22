#pragma once

#include <QPixmap>
#include <QPointF>

#include <array>

class QPainter;

/// Draws the streets, sidewalks, crossings, and building sprites behind the route.
class CityRenderer
{
public:
	CityRenderer();
	void draw(QPainter& painter, QPointF origin, double scale) const;

private:
	std::array<QPixmap, 3> m_buildingBitmaps;
};
