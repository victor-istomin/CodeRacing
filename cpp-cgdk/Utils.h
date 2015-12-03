#pragma once
#ifndef _DEBUG
#  ifndef NDEBUG
#    define NDEBUG
#  endif
#endif
#ifndef PI
#  define PI 3.14159265358979323846
#endif
#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#include "model/Game.h"
#include "model/World.h"
#include <vector>
#include <array>
#include <cmath>
#include <list>



struct PointD
{
	double x;
	double y;

	explicit PointD(double px = 0, double py = 0) : x(px), y(py)                   {}
	explicit PointD(const model::Unit& unit)      : x(unit.getX()), y(unit.getY()) {}
	static const double k_epsilon;

	PointD  operator*(double m)       const  { return PointD(x * m, y * m); }
	PointD  operator/(double m)       const  { return PointD(x / m, y / m); }
	PointD  operator+(const PointD& r) const { return PointD(x + r.x, y + r.y); }
	PointD  operator-(const PointD& r) const { return PointD(x - r.x, y - r.y); }

	PointD& operator*=(double m)             { x *= m; y *= m; return *this; }
	PointD& operator/=(double m)             { x /= m; y /= m; return *this; }

	double distanceTo(const PointD& p) const { PointD diff = *this - p; return std::hypot(diff.x, diff.y); }


	bool operator==(const PointD& p) const { return std::abs(x - p.x) < k_epsilon && std::abs(y - p.y) < k_epsilon; }
};

struct PointI
{
	int x;
	int y;

	explicit PointI(int px = 0, int py = 0) : x(px), y(py) {}
	explicit PointI(const PointD& p) : x(static_cast<int>(p.x)), y(static_cast<int>(p.y)) {}

	PointI& operator=(const PointI& right) { x = right.x; y = right.y; return *this; }

	PointI operator*(double m)       const { return PointI(static_cast<int>(x * m), static_cast<int>(y * m)); }
	PointI operator/(double m)       const { return PointI(static_cast<int>(x / m), static_cast<int>(y / m)); }
	PointI operator+(const PointI& r) const { return PointI(static_cast<int>(x + r.x), static_cast<int>(y + r.y)); }
	PointI operator-(const PointI& r) const { return PointI(static_cast<int>(x - r.x), static_cast<int>(y - r.y)); }

	double distanceTo(const PointI& p) const { PointI diff = *this - p; return std::hypot(diff.x, diff.y); }

	bool operator==(const PointI& p) const { return x == p.x && y == p.y; }
};

enum class RelativeTurn { TURN_NONE = 0, TURN_CLOCKWISE, TURN_COUNTER_CLOCKWISE, };

enum class AbsoluteDirection
{
	UNKNOWN = 0,
	// values order is important!
	LEFT, RIGHT,
	UP, DOWN,
};

struct TileNode
{
	struct Transition
	{
		TileNode*         m_cachedParent;
		AbsoluteDirection m_turnedDirection;   // which turn performed to move 'parent' -> 'this'

		Transition() : m_cachedParent(nullptr), m_turnedDirection(AbsoluteDirection::UNKNOWN) {}
		Transition(TileNode& from, TileNode& to);

		unsigned getCost(const TileNode& thisNode) const;
	};

	model::TileType m_type;
	PointI          m_pos;
	Transition      m_transition;
	bool            m_isWaypoint;

	TileNode() : m_type(model::UNKNOWN), m_pos(), m_transition() {}
};

struct TilePathNode
{
	typedef AbsoluteDirection AbsoluteTurn; // TODO - remove this typedef

	PointI       m_pos;
	AbsoluteTurn m_turnAbsolute;
	AbsoluteTurn m_turnAbsoluteFrom;
	RelativeTurn m_turnRelative;
	bool         m_isWaypoint;

	explicit TilePathNode(const TileNode& node);
};

typedef std::list<TilePathNode> Path;

