#pragma once
#include "Utils.h"

struct Node
{
	PointD m_pos;
	Node*  m_cachedParent;
	bool   m_isPassable;

	double m_gx; // g(x) = 'start -> this' cost
	double m_hx; // h(x) = 'this -> goal' heuristics estimate cost

	explicit Node(const PointD& p = PointD(0,0)) : m_pos(p), m_cachedParent(nullptr), m_gx(0), m_hx(0) {}
	double fx() const { return m_gx + m_hx; } // f(x) = g(x) + h(x)

	bool operator==(const Node& n) const { return m_pos == n.m_pos; }
};

class Map
{
	typedef std::vector< std::vector< model::TileType > > TilesXY;

	typedef std::vector<Node> Nodes;
	const static unsigned NODES_IN_TILE_AXIS = 10;  // nodes in tile by each axis

	TilesXY             m_tiles;
	Nodes               m_nodes;

	const model::Game*  m_game;
	const model::World* m_world;
	const int           m_tileSize;
	const PointD        m_tileCenter;

	const int    m_worldWidthNodes;
	const int    m_worldHeightNodes;
	const int    m_worldMartixSideNodes;
	const double m_worldPixelsInNode;

public:
	enum class Direction
	{
		UNKNOWN = 0,
		LEFT, RIGHT, UP, DOWN,
		LEFT_UP, LEFT_DOWN,
		RIGHT_UP, RIGHT_DOWN,
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

	Node* getNodePtr(const PointD& worldPoint) 
	{ 
		PointD nodePoint = worldPoint / m_worldPixelsInNode;
		return getNodePtr(nodePoint.x, nodePoint.y); 
	}

	Node* getNodePtr(int x, int y) 
	{ 
		size_t index = getNodeIndex(x, y); 
		return getNodePtr(index); 
	}

	Node* getNodePtr(size_t index)  { return &m_nodes[index]; }

	size_t worldSizeTiles() const { return m_world->getWidth() * m_world->getHeight(); }
	size_t worldSizeNodes() const { return m_nodes.size(); }

	model::TileType getTileType(int x, int y)   const { return m_tiles[x][y]; }
	PointD          getTileCorner(int x, int y) const { return PointD(x * m_tileSize, y * m_tileSize); }
	PointD          getTileCenter(int x, int y) const { return getTileCorner(x, y) + m_tileCenter; }

	Node pointToNode(const PointD& p)  const { return Node(p / m_worldPixelsInNode); }

	size_t getNodeIndex(int x, int y) const { return x * m_worldMartixSideNodes + y; }
	size_t nodeToIndex(const Node& n) const { return getNodeIndex(n.m_pos.x, n.m_pos.y); }

	PointD nodeToPoint(const Node& n)  const { return n.m_pos * m_worldPixelsInNode; }
	PointD nodeToTileXY(const Node& n) const { return nodeToPoint(n) / m_tileSize; }

	bool isNodePassable(const Node& node) const;
	bool isGoalPoint(const PointD& p, const PointD& goal) const; // TODO - check!
	PointD incrementNodeIndex(const PointD& point, Direction intcrementTo);

	double getTransitionCost(const Node& from, const Node& neighbour) const;
	double getHeuristicsTo(const Node& node, const PointD& goal) const;


	template <typename InitCallback>
	void fillNeighbors(const Node& node, InitCallback initNode) 
	{
		if (!isNodePassable(node))
			return;

		Direction directions[] = { Direction::UP, Direction::LEFT_UP, Direction::RIGHT_UP,
		                           Direction::DOWN, Direction::LEFT_DOWN, Direction::RIGHT_DOWN,
		                           Direction::LEFT, Direction::RIGHT };

		for (Direction d : directions)
		{
			PointD nextNodeIndex = incrementNodeIndex(node.m_pos, d);
			Node& next = * getNodePtr(nextNodeIndex.x, nextNodeIndex.y);

			if (next.m_isPassable)
				initNode(next);
		}
	}
};


