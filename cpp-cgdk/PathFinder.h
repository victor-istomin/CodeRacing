#pragma once
#include "Utils.h"
#include "Map.h"

#include <list>

class PathFinder
{
	Map& m_map;

public:
	typedef std::list<TilePathNode> Path;

	explicit PathFinder(Map& map)
		: m_map(map)
	{
	}

	Path getPath(const PointD& from, const PointD& to);

};
