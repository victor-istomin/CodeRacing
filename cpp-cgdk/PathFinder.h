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
		typedef TileNode::Transition::TurnDirection TurnAbsolute;
		enum TurnRelative { TURN_NONE = 0, TURN_CLOCKWISE, TURN_COUNTER_CLOCKWISE, };
		
		PointI       m_pos;
		TileNode::Transition::TurnDirection m_turnAbsolute;
		TurnRelative m_turnRelative;

		explicit TilePathNode(const TileNode& node);
	};

	typedef std::list<TilePathNode> Path;

	explicit PathFinder(Map& map)
		: m_map(map)
	{
	}

	Path getPath(const PointD& from, const PointD& to);

};
