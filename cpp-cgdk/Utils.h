#pragma once
#include "model/Game.h"
#include "model/World.h"
#include <vector>

struct Point
{
	double x;
	double y;

	explicit Point(double px = 0, double py = 0) : x(px), y(py) {}

	Point operator*(double m)       const { return Point(x * m, y*m); }
	Point operator+(const Point& r) const { return Point(x + r.x, y + r.y); }

};


class Map
{
	typedef std::vector< std::vector< model::TileType > > TilesXY;

	TilesXY             m_tiles;
	const model::Game*  m_game;
	const model::World* m_world;
	const double        m_tileSize;
	const Point         m_tileCenter;

public:
	Map(const model::Game& game, const model::World& world)
		: m_tiles(world.getTilesXY())
		, m_game(&game)
		, m_world(&world)
		, m_tileSize(m_game->getTrackTileSize())
		, m_tileCenter(m_game->getTrackTileSize() / 2, m_game->getTrackTileSize() / 2)
	{
	}

	void update(const model::Game& game, const model::World& world)
	{
		m_game  = &game;
		m_world = &world;
		m_tiles = world.getTilesXY();  // optimize and use reference or single array?
	}

	model::TileType getTileType(int x, int y)   const { return m_tiles[x][y]; }
	Point           getTileCorner(int x, int y) const { return Point(x * m_tileSize, y * m_tileSize); }
	Point           getTileCenter(int x, int y) const { return getTileCorner(x, y) + m_tileCenter; }
};

