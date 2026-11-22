#include "CityRenderer.h"

#include "CityMap.h"

#include <QPainter>
#include <QPolygonF>

namespace
{
	constexpr double roadHalfWidth = 8;

}

CityRenderer::CityRenderer()
	: m_buildingBitmaps{
		  QPixmap(":/map/app/assets/BuildingBlue.png"),
		  QPixmap(":/map/app/assets/BuildingRed.png"),
		  QPixmap(":/map/app/assets/BuildingGreen.png"),
	  }
{
}

void CityRenderer::draw(QPainter& painter, QPointF origin, double scale) const
{
	const auto screen = [&](nav::Point point) { return origin + QPointF(point.x * scale, -point.y * scale); };
	const auto area = [&](double x, double y, double w, double h) {
		return QRectF(screen({x, y + h}), QSizeF(w * scale, h * scale));
	};

	painter.fillRect(
		area(-roadHalfWidth, -roadHalfWidth, city::MapWidth + 2 * roadHalfWidth, city::MapHeight + 2 * roadHalfWidth),
		QColor("#323842"));

	// Chamfered sidewalks leave room for the crossings at each junction.
	painter.setPen(Qt::NoPen);
	painter.setRenderHint(QPainter::SmoothPixmapTransform);
	for (int row = 0; row < city::Rows; ++row)
	{
		for (int column = 0; column < city::Columns; ++column)
		{
			const double x = column * city::BlockPitch;
			const double y = row * city::BlockPitch;
			const QPolygonF sidewalk{
				screen({x + 8, y + 20}),
				screen({x + 20, y + 8}),
				screen({x + 84, y + 8}),
				screen({x + 96, y + 20}),
				screen({x + 96, y + 84}),
				screen({x + 84, y + 96}),
				screen({x + 20, y + 96}),
				screen({x + 8, y + 84}),
			};
			painter.setBrush(QColor("#78736a"));
			painter.drawPolygon(sidewalk);
			painter.fillRect(area(x + 32, y + 28, 40, 48), QColor("#404f45"));

			const std::size_t variant = (row + 2 * column) % m_buildingBitmaps.size();
			const std::array<QRectF, 4> roofs{
				area(x + 18, y + 61, 69, 29),
				area(x + 18, y + 14, 69, 28),
				area(x + 14, y + 38, 22, 29),
				area(x + 69, y + 38, 22, 29),
			};
			for (std::size_t i = 0; i < roofs.size(); ++i)
			{
				const auto& bitmap = m_buildingBitmaps[(variant + i) % m_buildingBitmaps.size()];
				painter.drawPixmap(roofs[i], bitmap, QRectF(bitmap.rect()));
			}
		}
	}

	const auto drawStreetMarkings = [&](bool horizontal, int streets, int blocks) {
		const QColor centerLine("#b9b1a0");
		const QColor crossing("#dedbd2");
		for (int street = 0; street <= streets; ++street)
		{
			const double center = street * city::BlockPitch;
			const auto marking = [&](double along, double across, double length, double width, const QColor& color) {
				painter.fillRect(horizontal ? area(along, center + across, length, width)
											: area(center + across, along, width, length),
								 color);
			};
			for (int block = 0; block < blocks; ++block)
			{
				const double start = block * city::BlockPitch;
				for (int offset = 24; offset <= 78; offset += 9)
				{
					marking(start + offset, -0.25, 4, 0.5, centerLine);
				}
				for (int stripe = -5; stripe <= 5; stripe += 2)
				{
					marking(start + 13, stripe, 4, 0.8, crossing);
					marking(start + city::BlockPitch - 17, stripe, 4, 0.8, crossing);
				}
			}
		}
	};
	drawStreetMarkings(true, city::Rows, city::Columns);
	drawStreetMarkings(false, city::Columns, city::Rows);
}
