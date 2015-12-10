#include "Map.h"
#include "PathFinder.h"

#include <cassert>
#include <cmath>
#include <map>
#include <algorithm>

const double Map::NORMAL_TURN_COST = 2;
const double Map::MIN_TURN_COST    = 1;

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

		typedef std::pair<TurnTuple, model::TileType> TurnsCornerPair;
		static const TurnsCornerPair turnsToCorner[] =
		{
			{ { AbsoluteDirection::UP,   AbsoluteDirection::RIGHT }, model::LEFT_TOP_CORNER },
			{ { AbsoluteDirection::LEFT, AbsoluteDirection::DOWN },  model::LEFT_TOP_CORNER },

			{ { AbsoluteDirection::UP,    AbsoluteDirection::LEFT},  model::RIGHT_TOP_CORNER },
			{ { AbsoluteDirection::RIGHT, AbsoluteDirection::DOWN }, model::RIGHT_TOP_CORNER },

			{ { AbsoluteDirection::LEFT,  AbsoluteDirection::UP },    model::LEFT_BOTTOM_CORNER },
			{ { AbsoluteDirection::DOWN,  AbsoluteDirection::RIGHT }, model::LEFT_BOTTOM_CORNER },

			{ { AbsoluteDirection::RIGHT, AbsoluteDirection::UP},    model::RIGHT_BOTTOM_CORNER },
			{ { AbsoluteDirection::DOWN,  AbsoluteDirection::LEFT},  model::RIGHT_BOTTOM_CORNER },
		};

		TurnTuple thisTurn = {turn.m_turnAbsoluteFrom, turn.m_turnAbsolute/*to*/};
		auto found = std::find_if(std::begin(turnsToCorner), std::end(turnsToCorner), [&thisTurn](const TurnsCornerPair& tupleCornerPair) 
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

double Map::getCostFromStart(TileNode& node) const
{
	double cost = 0;
	TileNode* parent = &node;// .m_transition.m_cachedParent;

	while (parent != nullptr)
	{
		cost += parent->m_transition.m_cachedTransitionCost;
		parent = parent->m_transition.m_cachedParent;
	}

	return cost;
}

double Map::getHeuristicsTo(const TileNode& node, const TileNode& goal) const
{
	PointI diff = node.m_pos - goal.m_pos;
	return std::hypot(diff.x, diff.y) * Map::MIN_TURN_COST; // TODO - cars, obstacles, angle, etc.
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
	, m_isZigzag(false)
	, m_cachedTransitionCost(0)
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

double TileNode::Transition::getCost(const Vec2d& selfSpeed, const TileNode& thisNode)
{
	static const double TURN_PENALTY                  = Map::NORMAL_TURN_COST;
	static const double TURN_180_PENALTY              = 4 * Map::NORMAL_TURN_COST;
	static const double WAYPOINT_OUT_OF_ORDER_PENALTY = Map::NORMAL_TURN_COST; // TODO - increase?     // it's not so good to go through incorrect waypoint
	static const double ZIGZAZ_TURN_PRIZE             = -Map::NORMAL_TURN_COST / 1.5;  // zigzag turn is slightly better then usual turn

	double cost = Map::NORMAL_TURN_COST;

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

	TileNode* prePrevious = m_cachedParent != nullptr ? m_cachedParent->m_transition.m_cachedParent : nullptr;
	bool isZigzag = m_isZigzag // already calculated
		|| ( prePrevious != nullptr
		  && prePrevious->m_transition.m_turnedDirection == m_turnedDirection
		  && m_cachedParent->m_transition.m_turnedDirection != m_turnedDirection );

	if (isZigzag)
	{
		cost += ZIGZAZ_TURN_PRIZE;
	}

	m_isZigzag = isZigzag;
	if (m_cachedParent != nullptr)
		m_cachedParent->m_transition.m_isZigzag = isZigzag;

	if (thisNode.m_isWaypoint)
		cost += WAYPOINT_OUT_OF_ORDER_PENALTY;

	// speed vector may become a bonus or penalty for transitions near start
	bool isMoving = selfSpeed.m_x != 0 || selfSpeed.m_y != 0;
	if (m_cachedParent != nullptr && isMoving)
	{
		int tilesFromStart = 0;
		Vec2d correctedSpeed = selfSpeed;
		TileNode* previousNode = m_cachedParent->m_transition.m_cachedParent;
		while (previousNode != nullptr)
		{
			correctedSpeed /= 2;  // further from start -> less penalty
			previousNode = previousNode->m_transition.m_cachedParent;

			if (++tilesFromStart > 2)
			{
				// too far, no sense to calculate
				correctedSpeed = Vec2d(0, 0);
				break;
			}
		}

		// remove "noise"
		if (std::abs(correctedSpeed.m_x) < 1)
			correctedSpeed.m_x = 0;
		if (std::abs(correctedSpeed.m_y) < 1)
			correctedSpeed.m_y = 0;

		// todo - use tilesFromStart?
		Vec2d turnVector = Vec2d::fromPoint(thisNode.m_pos - m_cachedParent->m_pos);
		double directionPrize = Vec2d::dot(correctedSpeed, turnVector) / selfSpeed.length(); // [-1; +1]

		// almost no penalty after log2(8) = 3 tiles
		static const double SPEED_VECTOR_PENALTY = -Map::NORMAL_TURN_COST * 16;  // negative prize * negative penalty = positive cost incerment
		static const double SPEED_VECTOR_PRIZE   = -Map::NORMAL_TURN_COST * 2;   // positive prize * negative penalty = negative cost increment

		double costIncrement = directionPrize > 0 ? directionPrize * SPEED_VECTOR_PRIZE : directionPrize * SPEED_VECTOR_PENALTY;
		double newCost = cost + costIncrement;
		cost = std::max(newCost, Map::MIN_TURN_COST);
	}

	return cost;		
}
