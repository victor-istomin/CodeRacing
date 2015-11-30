#include "Map.h"
#include "PathFinder.h"

#include <cassert>
#include <cmath>
#include <map>

Map::Map(const model::Game& game, const model::World& world)
	: m_tiles(world.getTilesXY())
	, m_game(&game)
	, m_world(&world)

	, m_tileSize((int)m_game->getTrackTileSize())
	, m_tileCenter((int)m_game->getTrackTileSize() / 2, (int)m_game->getTrackTileSize() / 2)
	, m_worldWidthTiles(world.getWidth())
	, m_worldHeightTiles(world.getHeight())
{

	m_tileNodes.resize(m_worldWidthTiles * m_worldHeightTiles);
	updateNodes();
}

void Map::updateNodes()
{
	for (int x = 0; x < m_worldWidthTiles; ++x)
	{
		for (int y = 0; y < m_worldHeightTiles; ++y)
		{
			TileNode& tileNode = m_tileNodes[getTileNodeIndex(x, y)];
			tileNode.m_type = getTileType(x, y);
			tileNode.m_pos = PointI(x, y);
			tileNode.m_transition = TileNode::Transition();
			tileNode.m_isWaypoint = false;
			// TODO - cars, etc...
		}
	}

	for (const auto& waypoint : m_world->getWaypoints())
	{
		TileNode& tileNode = m_tileNodes[getTileNodeIndex(waypoint[0], waypoint[1])];
		tileNode.m_isWaypoint = true;
	}
}

PointD Map::getTurnOuterCorner(int x, int y, const TilePathNode& turn) const
{
	static const double displacement = m_game->getTrackTileMargin() + std::min(m_game->getCarWidth(), m_game->getCarHeight()) / 2;
	static const double bigDisplacement = m_game->getTrackTileSize() - displacement;

	static const std::map<model::TileType, PointD> displacementMap = 
	{
		{ model::LEFT_TOP_CORNER,     PointD(displacement,    displacement) },
		{ model::LEFT_BOTTOM_CORNER,  PointD(displacement,    bigDisplacement) },
		{ model::RIGHT_TOP_CORNER,    PointD(bigDisplacement, displacement) },
		{ model::RIGHT_BOTTOM_CORNER, PointD(bigDisplacement, bigDisplacement) },

		// others not yet implemented
	};

	model::TileType tileType = getTileType(x, y);

	switch (tileType)
	{
	case model::LEFT_HEADED_T:
	case model::RIGHT_HEADED_T:
	case model::TOP_HEADED_T:
	case model::BOTTOM_HEADED_T:
	case model::CROSSROADS:
	{
		// T-turn or crossroads handling is similar to corresponding usual turn

		struct TurnTuple 
		{
			TilePathNode::AbsoluteTurn from, to; 
			bool operator==(const TurnTuple&r) const { return from == r.from && to == r.to; }
		};

		static const std::pair<TurnTuple, model::TileType> turnsToCorner[] =
		{
			{ { AbsoluteDirection::UP,   AbsoluteDirection::RIGHT }, model::LEFT_TOP_CORNER },
			{ { AbsoluteDirection::LEFT, AbsoluteDirection::DOWN },  model::LEFT_TOP_CORNER },

			{ { AbsoluteDirection::UP,    AbsoluteDirection::LEFT},  model::RIGHT_TOP_CORNER },
			{ { AbsoluteDirection::RIGHT, AbsoluteDirection::DOWN }, model::RIGHT_TOP_CORNER },

			{ { AbsoluteDirection::RIGHT, AbsoluteDirection::UP},    model::LEFT_BOTTOM_CORNER },
			{ { AbsoluteDirection::DOWN,  AbsoluteDirection::RIGHT}, model::LEFT_BOTTOM_CORNER },

			{ { AbsoluteDirection::LEFT,  AbsoluteDirection::UP },    model::LEFT_BOTTOM_CORNER },
			{ { AbsoluteDirection::DOWN,  AbsoluteDirection::RIGHT }, model::LEFT_BOTTOM_CORNER },

			{ { AbsoluteDirection::LEFT,  AbsoluteDirection::UP },    model::LEFT_BOTTOM_CORNER },
			{ { AbsoluteDirection::DOWN,  AbsoluteDirection::RIGHT }, model::LEFT_BOTTOM_CORNER },
		};

		TurnTuple thisTurn = {turn.m_turnAbsoluteFrom, turn.m_turnAbsolute/*to*/};
		auto found = std::find_if(std::begin(turnsToCorner), std::end(turnsToCorner), [&thisTurn](const auto& tupleCornerPair) 
		{
			return tupleCornerPair.first == thisTurn;
		});

		if (found != std::end(turnsToCorner))
		{
			tileType = found->second;   // assume this crossroad to be usual turn
		}		

		break;
	}
	default:
		break;
	}

	auto displacementIt = displacementMap.find(tileType);
	return getTileCorner(x, y) + (displacementIt != displacementMap.end() ? displacementIt->second : m_tileCenter);
}

bool Map::incrementTileNodeIndex(const TileNode& initial, Direction incrementTo, PointI& result)
{
	assert(incrementTo != Direction::UNKNOWN);

	// TODO - validate turn
	bool isValidDirection = true;
	switch (initial.m_type)
	{
	case model::EMPTY:
		assert(initial.m_type != model::EMPTY);  // no way here...
		break;

	case model::VERTICAL:
		isValidDirection = incrementTo == Direction::UP || incrementTo == Direction::DOWN;
		break;

	case model::HORIZONTAL:
		isValidDirection = incrementTo == Direction::LEFT || incrementTo == Direction::RIGHT;
		break;

	case model::LEFT_TOP_CORNER:
		isValidDirection = incrementTo == Direction::DOWN || incrementTo == Direction::RIGHT;
		break;

	case model::RIGHT_TOP_CORNER:
		isValidDirection = incrementTo == Direction::DOWN || incrementTo == Direction::LEFT;
		break;

	case model::LEFT_BOTTOM_CORNER:
		isValidDirection = incrementTo == Direction::UP || incrementTo == Direction::RIGHT;
		break;

	case model::RIGHT_BOTTOM_CORNER:
		isValidDirection = incrementTo == Direction::UP || incrementTo == Direction::LEFT;
		break;

	case model::LEFT_HEADED_T:
		isValidDirection = incrementTo != Direction::RIGHT;
		break;

	case model::RIGHT_HEADED_T:
		isValidDirection = incrementTo != Direction::LEFT;
		break;

	case model::TOP_HEADED_T:
		isValidDirection = incrementTo != Direction::DOWN;
		break;

	case model::BOTTOM_HEADED_T:
		isValidDirection = incrementTo != Direction::UP;
		break;

	case model::CROSSROADS:
		isValidDirection = true; // any OK
		break;

	case model::UNKNOWN:
	case model::_TILE_TYPE_COUNT_:
	default:
		isValidDirection = true;
		assert(0 && "no idea yet");
		break;
	}

	const PointI& point = initial.m_pos;

	auto incremented = point;
	if (incrementTo == Direction::DOWN)
		incremented.y++;

	if (incrementTo == Direction::UP)
		incremented.y--;

	if (incrementTo == Direction::LEFT)
		incremented.x--;

	if (incrementTo == Direction::RIGHT)
		incremented.x++;

	result = incremented;
	return isValidDirection 
		&& incremented.x >= 0 && incremented.x < m_worldWidthTiles && incremented.y >= 0 && incremented.y < m_worldHeightTiles;
}

double Map::getCostFromStart(const TileNode& node) const
{
	unsigned cost = 0;
	const TileNode* parent = node.m_transition.m_cachedParent;

	while (parent != nullptr)
	{
		cost += parent->m_transition.getCost(node);
		parent = parent->m_transition.m_cachedParent;
	}

	return cost;
}

double Map::getHeuristicsTo(const TileNode& node, const TileNode& goal) const
{
	PointI diff = node.m_pos - goal.m_pos;
	return std::hypot(diff.x, diff.y); // TODO - cars, obstacles, angle, etc.
}

void Map::resetPathFinderCaches()
{
	const TileNode::Transition emptyTransition;

	for (TileNode& node : m_tileNodes)
		node.m_transition = emptyTransition;
}

TileNode::Transition::Transition(TileNode& from, TileNode& to)
	: m_cachedParent(&from)
	, m_turnedDirection(AbsoluteDirection::UNKNOWN)
{
	if (from.m_pos.x > to.m_pos.x)
		m_turnedDirection = AbsoluteDirection::LEFT;
	if (from.m_pos.x < to.m_pos.x)
		m_turnedDirection = AbsoluteDirection::RIGHT;
	if (from.m_pos.y > to.m_pos.y)
		m_turnedDirection = AbsoluteDirection::UP;
	if (from.m_pos.y < to.m_pos.y)
		m_turnedDirection = AbsoluteDirection::DOWN;

	assert(m_turnedDirection != AbsoluteDirection::UNKNOWN);
}

unsigned TileNode::Transition::getCost(const TileNode& thisNode) const
{
	static const unsigned TURN_PENALTY                  = Map::NORMAL_TURN_COST;
	static const unsigned TURN_180_PENALTY              = 4 * Map::NORMAL_TURN_COST;
	static const unsigned WAYPOINT_OUT_OF_ORDER_PENALTY = Map::NORMAL_TURN_COST;      // it's not so good to go through incorrect waypoint
	static const unsigned ZIGZAZ_TURN_PRIZE             = Map::NORMAL_TURN_COST / 2;  // zigzag turn is better then usual turn

	unsigned cost = Map::NORMAL_TURN_COST;

	AbsoluteDirection previousTurn = m_cachedParent == nullptr ? m_turnedDirection/*assume no change*/ : m_cachedParent->m_transition.m_turnedDirection;
	if (m_turnedDirection != previousTurn)
		cost += TURN_PENALTY;

	AbsoluteDirection minDirection = std::min(m_turnedDirection, previousTurn);
	AbsoluteDirection maxDirection = std::max(m_turnedDirection, previousTurn);

	if ((minDirection == AbsoluteDirection::LEFT && maxDirection == AbsoluteDirection::RIGHT)
	 || (minDirection == AbsoluteDirection::UP   && maxDirection == AbsoluteDirection::DOWN))
	{
		cost += TURN_180_PENALTY;
	}

	/*static const std::pair<TurnDirection, TurnDirection> directions180[] = 
	{
		{TurnDirection::LEFT,  TurnDirection::RIGHT},
		{TurnDirection::RIGHT, TurnDirection::LEFT},
		{TurnDirection::DOWN,  TurnDirection::UP},
		{TurnDirection::UP,    TurnDirection::DOWN}
	};*/

	TileNode* prePrevious = m_cachedParent != nullptr ? m_cachedParent->m_transition.m_cachedParent : nullptr;
	if (prePrevious != nullptr && prePrevious->m_transition.m_turnedDirection == m_turnedDirection)
	{
		cost -= ZIGZAZ_TURN_PRIZE;
	}

	if (thisNode.m_isWaypoint)
		cost += WAYPOINT_OUT_OF_ORDER_PENALTY;

	return cost;		
}
