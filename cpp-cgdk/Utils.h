#pragma once
#include "model/Game.h"

struct Point
{
	double x;
	double y;

	explicit Point(double px = 0, double py = 0) : x(px), y(py) {}
	
	static Point fromTileIndex(const model::Game& game, int x, int y)
	{
		return Point((x + 0.5) * game.getTrackTileSize(), (y + 0.5) * game.getTrackTileSize());
	}
};

