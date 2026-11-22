#pragma once

#include "CityRenderer.h"
#include "nav/RouteTracker.h"

#include <QWidget>

#include <vector>

/// Displays the planned route, recent GPS fixes, and current tracking frame.
class MapCanvas : public QWidget
{
public:
	MapCanvas();

	void setFrame(nav::Point truth, nav::Point gps, const nav::Match& match);
	void clearObservations();

protected:
	void paintEvent(QPaintEvent* event) override;

private:
	CityRenderer m_cityRenderer;
	nav::Point m_truth{};
	nav::Point m_gps{};
	nav::Match m_match{};
	std::vector<nav::Point> m_observations;
};
