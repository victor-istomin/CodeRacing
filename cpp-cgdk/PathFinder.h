#pragma once
#include "Utils.h"
#include "Map.h"

#include <list>

class PathFinder
{
	Map& m_map;

public:
	struct TilePathNode
	{
		typedef TileNode::Transition::TurnDirection AbsoluteTurn;
		
		PointI       m_pos;
		AbsoluteTurn m_turnAbsolute;
		RelativeTurn m_turnRelative;
		bool         m_isWaypoint;

		explicit TilePathNode(const TileNode& node);
	};

	typedef std::list<TilePathNode> Path;

	explicit PathFinder(Map& map)
		: m_map(map)
	{
	}

	Path getPath(const PointD& from, const PointD& to);

};
