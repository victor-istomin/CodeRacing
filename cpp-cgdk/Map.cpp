#include "Map.h"
#include <cassert>
#include <cmath>

Map::Map(const model::Game& game, const model::World& world)
	: m_tiles(world.getTilesXY())
	//, m_nodes()
	, m_game(&game)
	, m_world(&world)

	, m_tileSize((int)m_game->getTrackTileSize())
	, m_tileCenter((int)m_game->getTrackTileSize() / 2, (int)m_game->getTrackTileSize() / 2)
	, m_worldWidthTiles(world.getWidth())
	, m_worldHeightTiles(world.getHeight())

	//, m_worldWidthNodes((int)world.getWidth() * NODES_IN_TILE_AXIS)
	//, m_worldHeightNodes((int)world.getHeight() * NODES_IN_TILE_AXIS)
	//, m_worldPixelsInNode(m_tileSize / NODES_IN_TILE_AXIS)
	//, m_worldMartixSideNodes(std::max(m_worldWidthNodes, m_worldHeightNodes))
{
	//m_nodes.resize(m_worldMartixSideNodes * m_worldMartixSideNodes);

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
			// TODO - cars, etc...
		}
	}
}

/*bool Map::isNodePassable(const Node& node) const
{
	if (node.m_pos.x < 0 || node.m_pos.y < 0 
		|| node.m_pos.x >= m_worldWidthNodes
		|| node.m_pos.y >= m_worldHeightNodes)
			return false;

	PointD           tileXY   = nodeToTileXY(node);
	model::TileType  tileType = getTileType(tileXY.x, tileXY.y);
	PointD           positionInTile = nodeToPoint(node) - getTileCorner(tileXY.x, tileXY.y);

	assert(positionInTile.x <= m_tileSize && positionInTile.y <= m_tileSize);

	bool isPassable = true;
	const int carMargin        = std::min(m_game->getCarWidth(), m_game->getCarHeight()) / 2;
	const int stoneMargin      = carMargin + m_game->getTrackTileMargin();

	const int tileLeftMargin   = m_game->getTrackTileMargin() + carMargin;
	const int tileRightMargin  = m_tileSize - m_game->getTrackTileMargin() - carMargin;
	const int tileTopMargin    = m_game->getTrackTileMargin() + carMargin;
	const int tileBottomMargin = m_tileSize - m_game->getTrackTileMargin() - carMargin;

	const PointD rightBottomStone = PointD(m_tileSize, m_tileSize);
	const PointD leftBottomStone  = PointD(0, m_tileSize);
	const PointD rightTopStone    = PointD(m_tileSize, 0);
	const PointD leftTopStone     = PointD(0, 0);

	switch (tileType)
	{
	case model::EMPTY:
		isPassable = false;
		break;

	case model::VERTICAL:
		isPassable = positionInTile.x >= tileLeftMargin && positionInTile.x <= tileRightMargin;
		break;

	case model::HORIZONTAL:
		isPassable = positionInTile.y >= tileTopMargin && positionInTile.y <= tileBottomMargin;
		break;

	case model::LEFT_TOP_CORNER:
		isPassable = positionInTile.x >= tileLeftMargin && positionInTile.y >= tileTopMargin
			&& positionInTile.distanceTo(rightBottomStone) >= stoneMargin;
		break;

	case model::RIGHT_TOP_CORNER:
		isPassable = positionInTile.x <= tileRightMargin && positionInTile.y >= tileTopMargin
			&& positionInTile.distanceTo(leftBottomStone) >= stoneMargin;
		break;

	case model::LEFT_BOTTOM_CORNER:
		isPassable = positionInTile.x >= tileLeftMargin && positionInTile.y <= tileBottomMargin
			&& positionInTile.distanceTo(rightTopStone) >= stoneMargin;
		break;

	case model::RIGHT_BOTTOM_CORNER:
		isPassable = positionInTile.x <= tileRightMargin && positionInTile.y <= tileBottomMargin
			&& positionInTile.distanceTo(leftTopStone) >= stoneMargin;
		break;

	case model::LEFT_HEADED_T:
		isPassable = positionInTile.x <= tileRightMargin
			&& positionInTile.distanceTo(leftTopStone) >= stoneMargin
			&& positionInTile.distanceTo(leftBottomStone) >= stoneMargin;
		break;

	case model::RIGHT_HEADED_T:
		isPassable = positionInTile.x >= tileLeftMargin
			&& positionInTile.distanceTo(rightTopStone) >= stoneMargin
			&& positionInTile.distanceTo(rightBottomStone) >= stoneMargin;
		break;

	case model::TOP_HEADED_T:
		isPassable = positionInTile.y <= tileBottomMargin
			&& positionInTile.distanceTo(leftTopStone) >= stoneMargin
			&& positionInTile.distanceTo(rightTopStone) >= stoneMargin;
		break;

	case model::BOTTOM_HEADED_T:
		isPassable = positionInTile.y >= tileTopMargin
			&& positionInTile.distanceTo(leftBottomStone) >= stoneMargin
			&& positionInTile.distanceTo(rightBottomStone) >= stoneMargin;
		break;

	case model::CROSSROADS:
		isPassable = positionInTile.distanceTo(leftTopStone) >= stoneMargin
                  && positionInTile.distanceTo(rightTopStone) >= stoneMargin
		          && positionInTile.distanceTo(leftBottomStone) >= stoneMargin
		          && positionInTile.distanceTo(rightBottomStone) >= stoneMargin;
		break;

	case model::UNKNOWN: // TODO
	default:
		break;
	}

	return isPassable;
}*/

/*bool Map::isGoalPoint(const PointD& p, const PointD& goal) const
{
	return p.distanceTo(goal) <= m_worldPixelsInNode;
}
*/
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
		cost += parent->m_transition.getCost();
		parent = parent->m_transition.m_cachedParent;
	}

	return cost;
}

/*double Map::getTransitionCost(const Node& from, const Node& neighbour) const
{
	PointD diff = from.m_pos - neighbour.m_pos;
	return std::hypot(diff.x, diff.y); // TODO - cars, obstacles, angle, etc.
}*/

double Map::getHeuristicsTo(const TileNode& node, const TileNode& goal) const
{
	PointI diff = node.m_pos - goal.m_pos;
	return std::hypot(diff.x, diff.y); // TODO - cars, obstacles, angle, etc.
}

TileNode::Transition::Transition(TileNode& from, TileNode& to)
	: m_cachedParent(&from)
	, m_turnedDirection(UNKNOWN)
{
	if (from.m_pos.x > to.m_pos.x)
		m_turnedDirection = LEFT;
	if (from.m_pos.x < to.m_pos.x)
		m_turnedDirection = RIGHT;
	if (from.m_pos.y > to.m_pos.y)
		m_turnedDirection = UP;
	if (from.m_pos.y < to.m_pos.y)
		m_turnedDirection = DOWN;

	assert(m_turnedDirection != UNKNOWN);
}

unsigned TileNode::Transition::getCost() const
{
	static const unsigned NORMAL_COST  = 1;
	static const unsigned TURN_PENALTY = 1;

	unsigned cost = NORMAL_COST;

	TurnDirection previousTurn = m_cachedParent == nullptr ? m_turnedDirection/*assume no change*/ : m_cachedParent->m_transition.m_turnedDirection;
	if (m_turnedDirection != previousTurn)
		cost += TURN_PENALTY;

	return cost;		
}
