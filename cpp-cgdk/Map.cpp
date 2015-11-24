#include "Map.h"
#include <cassert>
#include <cmath>

Map::Map(const model::Game& game, const model::World& world) 
	: m_tiles(world.getTilesXY())
	, m_nodes()
	, m_game(&game)
	, m_world(&world)

	, m_tileSize((int)m_game->getTrackTileSize())
	, m_tileCenter((int)m_game->getTrackTileSize() / 2, (int)m_game->getTrackTileSize() / 2)

	, m_worldWidthNodes((int)world.getWidth() * NODES_IN_TILE_AXIS)
	, m_worldHeightNodes((int)world.getHeight() * NODES_IN_TILE_AXIS)
	, m_worldPixelsInNode(m_tileSize / NODES_IN_TILE_AXIS)
	, m_worldMartixSideNodes(std::max(m_worldWidthNodes, m_worldHeightNodes))
{
	m_nodes.resize(m_worldMartixSideNodes * m_worldMartixSideNodes);

	updateNodes();
}

void Map::updateNodes()
{
	for (size_t x = 0; x < m_worldWidthNodes; ++x)
	{
		for (size_t y = 0; y < m_worldHeightNodes; ++y)
		{
			Node& node = m_nodes[getNodeIndex(x, y)];
			node.m_pos = PointD(x, y);
			node.m_isPassable = isNodePassable(node);
			node.m_gx = node.m_hx = 0;
			node.m_cachedParent = nullptr;
			// TODO - cars, etc...
		}
	}
}

bool Map::isNodePassable(const Node& node) const
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
}

bool Map::isGoalPoint(const PointD& p, const PointD& goal) const
{
	return p.distanceTo(goal) <= m_worldPixelsInNode;
}

PointD Map::incrementNodeIndex(const PointD& point, Direction intcrementTo)
{
	assert(intcrementTo != Direction::UNKNOWN);

	PointD incremented = point;
	if (intcrementTo == Direction::DOWN || intcrementTo == Direction::LEFT_DOWN || intcrementTo == Direction::RIGHT_DOWN)
		incremented.y++;

	if (intcrementTo == Direction::UP || intcrementTo == Direction::LEFT_UP || intcrementTo == Direction::RIGHT_UP)
		incremented.y--;

	if (intcrementTo == Direction::LEFT || intcrementTo == Direction::LEFT_DOWN || intcrementTo == Direction::LEFT_UP)
		incremented.x--;

	if (intcrementTo == Direction::RIGHT || intcrementTo == Direction::RIGHT_DOWN || intcrementTo == Direction::RIGHT_UP)
		incremented.x++;

	return incremented;
}

double Map::getTransitionCost(const Node& from, const Node& neighbour) const
{
	PointD diff = nodeToPoint(from) - nodeToPoint(neighbour);
	return std::hypot(diff.x, diff.y); // TODO - cars, obstacles, angle, etc.
}

double Map::getHeuristicsTo(const Node& node, const PointD& goal) const
{
	PointD diff = nodeToPoint(node) - goal;
	return std::hypot(diff.x, diff.y); // TODO - cars, obstacles, angle, etc.
}

