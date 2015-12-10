#pragma once
#include "Utils.h"

class Map
{
	typedef std::vector< std::vector< model::TileType > > TilesXY;
	typedef std::vector<TileNode> TileNodes;

	TilesXY   m_tiles;
	TileNodes m_tileNodes;

	const model::Game*  m_game;
	const model::World* m_world;
	const int           m_tileSize;
	const PointD        m_tileCenter;
	const int           m_worldWidthTiles;
	const int           m_worldHeightTiles;

public:
	static const double NORMAL_TURN_COST;  // todo - check that this values used elsewhere instead of magic '1'!
	static const double MIN_TURN_COST;

	enum class Direction
	{
		UNKNOWN = 0,
		LEFT, RIGHT, UP, DOWN,
	//	LEFT_UP, LEFT_DOWN,
	//	RIGHT_UP, RIGHT_DOWN,
	};

	Map(const model::Game& game, const model::World& world);

	void updateNodes();

	void update(const model::Game& game, const model::World& world)
	{
		m_game  = &game;
		m_world = &world;
		m_tiles = world.getTilesXY();  // optimize and use reference or single array?

		updateNodes();
	}

	size_t worldSizeTiles() const { return m_world->getWidth() * m_world->getHeight(); }

	model::TileType getTileType(int x, int y)   const { return m_tiles[x][y]; }
	PointI getTileIndex(const PointD& p)        const { return PointI(p / m_tileSize); }
	PointD getTileCorner(int x, int y)          const { return PointD(x * m_tileSize, y * m_tileSize); }
	PointD getTileCenter(int x, int y)          const { return getTileCorner(x, y) + m_tileCenter; }
	PointD getTurnOuterCorner(int x, int y, const TilePathNode& turn) const;
	
	size_t getTileNodeIndex(int x, int y) const 
	{ 
		return x * m_worldHeightTiles + y; 
	}

	size_t getTileNodeIndex(PointD worldPoint) 
	{ 
		worldPoint /= m_tileSize; 
		return getTileNodeIndex(static_cast<int>(worldPoint.x), static_cast<int>(worldPoint.y)); 
	}

	bool incrementTileNodeIndex(const TileNode& initial, Direction intcrementTo, PointI& result);
	TileNode* getTileNodePtr(size_t index) { return &m_tileNodes[index]; }

	double getCostFromStart(TileNode& node) const;
	double getHeuristicsTo(const TileNode& node, const TileNode& goal) const;

	void resetPathFinderCaches(); // reset nodes parents, etc.

	template <typename InitCallback>
	void fillNeighbors(TileNode& node, InitCallback initNode)
	{
		static const Direction directions[] = 
		{ 
			Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT,
		};

		double parentCostFromStart = getCostFromStart(node);

		for (Direction d : directions)
		{
			PointI nextNodeIndex;
			if (!incrementTileNodeIndex(node, d, nextNodeIndex))
				continue;

			TileNode* next = getTileNodePtr(getTileNodeIndex(nextNodeIndex.x, nextNodeIndex.y));
			initNode(next, parentCostFromStart);
		}
	}
};


