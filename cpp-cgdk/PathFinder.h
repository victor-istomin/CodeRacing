#pragma once
#include "Utils.h"
#include "Map.h"
#include "Vec2.h"

#include <list>

class PathFinder
{
	Map& m_map;

public:

	explicit PathFinder(Map& map)
		: m_map(map)
	{
	}

	Path getPath(const Vec2d& selfSpeed, const PointD& from, const PointD& to);

};
