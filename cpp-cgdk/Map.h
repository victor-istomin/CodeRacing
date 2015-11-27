#pragma once
#include "Utils.h"
/*struct Node
{
	PointD m_pos;
	Node*  m_cachedParent;
	bool   m_isPassable;

	//double m_gx; // g(x) = 'start -> this' cost
	double m_cachedTransitionCost;
	double m_hx; // h(x) = 'this -> goal' heuristics estimate cost

	explicit Node(const PointD& p = PointD(0,0)) : m_pos(p), m_cachedParent(nullptr), m_cachedTransitionCost(0), m_hx(0) {}

	bool operator==(const Node& n) const { return m_pos == n.m_pos; }
};*/

struct TileNode
{
	struct Transition
	{
		enum TurnDirection
		{
			UNKNOWN = 0,
			LEFT, RIGHT, UP, DOWN,
		};

		TileNode*     m_cachedParent;
		TurnDirection m_turnedDirection;   // which turn performed to move 'parent' -> 'this'

		Transition() : m_cachedParent(nullptr), m_turnedDirection(UNKNOWN) {}
		Transition(TileNode& from, TileNode& to);

		unsigned getCost() const;
	};

	model::TileType m_type;
	PointI          m_pos;
	Transition      m_transition;

	TileNode() : m_type(model::UNKNOWN), m_pos(), m_transition() {}
};

class Map
{
	typedef std::vector< std::vector< model::TileType > > TilesXY;
	typedef std::vector<TileNode> TileNodes;

	//typedef std::vector<Node> Nodes;
	//const static unsigned NODES_IN_TILE_AXIS = 4;  // nodes in tile by each axis

	TilesXY   m_tiles;
	TileNodes m_tileNodes;
	//Nodes               m_nodes;

	const model::Game*  m_game;
	const model::World* m_world;
	const int           m_tileSize;
	const PointD        m_tileCenter;
	const int           m_worldWidthTiles;
	const int           m_worldHeightTiles;

	//const int    m_worldWidthNodes;
	//const int    m_worldHeightNodes;
	//const int    m_worldMartixSideNodes;
	//const double m_worldPixelsInNode;

public:
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

	/*
	Node* getNodePtr(const PointD& worldPoint) 
	{ 
		PointD nodePoint = worldPoint / m_worldPixelsInNode;
		return getNodePtr(nodePoint.x, nodePoint.y); 
	}

	Node* getNodePtr(int x, int y) 
	{ 
		size_t index = getNodeIndex(x, y); 
		return getNodePtr(index); 
	}*/


	//Nodes& nodes() { return m_nodes;  }
	

	size_t worldSizeTiles() const { return m_world->getWidth() * m_world->getHeight(); }
	//size_t worldSizeNodes() const { return m_nodes.size(); }

	model::TileType getTileType(int x, int y)   const { return m_tiles[x][y]; }
	PointD          getTileCorner(int x, int y) const { return PointD(x * m_tileSize, y * m_tileSize); }
	PointD          getTileCenter(int x, int y) const { return getTileCorner(x, y) + m_tileCenter; }

	//Node pointToNode(const PointD& p)  const { return Node(p / m_worldPixelsInNode); }

	
	//size_t getNodeIndex(int x, int y) const { return x * m_worldMartixSideNodes + y; }
	//size_t nodeToIndex(const Node& n) const { return getNodeIndex(n.m_pos.x, n.m_pos.y); }

	//PointD nodeToPoint(const Node& n)  const { return n.m_pos * m_worldPixelsInNode; }
	//PointD nodeToTileXY(const Node& n) const { return nodeToPoint(n) / m_tileSize; }

	//bool isNodePassable(const Node& node) const;
	//bool isGoalPoint(const PointD& p, const PointD& goal) const; // TODO - check!
	size_t getTileNodeIndex(int x, int y) const { return x * m_worldWidthTiles + y; }
	size_t getTileNodeIndex(PointD worldPoint) { worldPoint /= m_tileSize; return getTileNodeIndex(static_cast<int>(worldPoint.x), static_cast<int>(worldPoint.y)); }

	bool incrementTileNodeIndex(const TileNode& initial, Direction intcrementTo, PointI& result);
	TileNode* getTileNodePtr(size_t index) { return &m_tileNodes[index]; }


	double getCostFromStart(const TileNode& node) const;
	//double getTransitionCost(const TileNode& from, const TileNode& neighbour) const;
	double getHeuristicsTo(const TileNode& node, const TileNode& goal) const;


	template <typename InitCallback>
	void fillNeighbors(const TileNode& node, InitCallback initNode) 
	{
		static const Direction directions[] = 
		{ 
			Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT,
			//Direction::LEFT_UP, Direction::RIGHT_UP, Direction::LEFT_DOWN, Direction::RIGHT_DOWN,		     
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


