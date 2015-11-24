#pragma once
#ifdef _DEBUG
# include "Debug.h"
#endif

#include <string>
#include <cstdint>

#include "model/World.h"
#include "model/Car.h"
#include "model/Move.h"
#include "Utils.h"
#include "Map.h"
#include "PathFinder.h"

class DebugVisualizer
{

	Debug m_render;

public:
	DebugVisualizer() {}

	void preRenderStart() { m_render.beginPre(); }
	void preRenderFinish() { m_render.endPre(); }

	void renderWypoints(const Map& map, const model::Game& game, const model::World& world, const model::Car& self)
	{
		for (size_t index = 0; index < world.getWaypoints().size(); ++index)
		{
			int32_t color = index == (size_t)self.getNextWaypointIndex() ? 0xDDDDFF : 0xDDDDDD;
			const int xIdx = world.getWaypoints()[index][0];
			const int yIdx = world.getWaypoints()[index][1];
			const PointD topLeft = map.getTileCorner(xIdx, yIdx);
			const PointD center = map.getTileCenter(xIdx, yIdx);

			m_render.fillRect(topLeft.x, topLeft.y, topLeft.x + game.getTrackTileSize(), topLeft.y + game.getTrackTileSize(), color);
			m_render.text(center.x, center.y, std::to_string(index).c_str());
		}
	}

	void renderMoveTarget(const model::Car& self, const model::Move& move, const PointD& destination)
	{
		int32_t color = 0x8888BB;

		if (move.isUseNitro() || self.getRemainingNitroTicks() > 0)
			color = 0xBB8888;
		else if (move.isBrake())
			color = 0x88BB88;

		m_render.line(self.getX(), self.getY(), destination.x, destination.y, color);
		m_render.fillCircle(destination.x, destination.y, std::hypot(self.getWidth(), self.getHeight()) / 2, color);
	}

	void renderPath(PathFinder& pathFinder, const model::Car& self, const PointD& destinateion)
	{
		PointD start = PointD(PointD(self.getX(), self.getY()));

		PathFinder::Path path = pathFinder.getPath(start, PointD(destinateion));
		const PointD* previous = nullptr;
		for (const PointD& step : path)
		{
			if (previous != nullptr)
			{
				m_render.line(previous->x, previous->y, step.x, step.y, 0x008800);
			}

			m_render.circle(step.x, step.y, 50, 0x008800);
			previous = &step;
		}

	}

};
