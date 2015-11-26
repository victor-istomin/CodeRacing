#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "Utils.h"
#include <cmath>
#include <cstdint>
#include <numeric>
#include <type_traits>
#include <memory>

#ifdef _DEBUG
# include "DebugVisualizer.h"
#else
struct DebugVisualizer
{
	void preRenderStart() { }
	void preRenderFinish() { }

	void renderWypoints(const Map&, const model::Game&, const model::World&, const model::Car&) {}
	void renderMoveTarget(const model::Car&, const model::Move&, const Point&) {}
};
#endif

struct DebugMessage
{
	DebugVisualizer&    m_v;

	const Map&          m_map;
	const model::Car&   m_self;
	const model::World& m_world;
	const model::Game&  m_game;
	const model::Move&  m_move;

	Point m_destination;

	DebugMessage(DebugVisualizer& v, const Map& map, const model::Car& self, const model::World& world, const model::Game& game, const model::Move& move)
		: m_v(v), m_map(map), m_self(self), m_world(world), m_game(game), m_move(move)
	{
	}

	~DebugMessage()
	{
		// pre-render block
		m_v.preRenderStart();
		m_v.renderWypoints(m_map, m_game, m_world, m_self);
		m_v.renderMoveTarget(m_self, m_move, m_destination);
		m_v.preRenderFinish();
	}
};

class MyStrategy : public Strategy 
{
	static const int CORNERING_SPEED_CAREFUL = 7;
	static const int CORNERING_SPEED_REGULAR = 10;
	static const int CORNERING_SPEED_OILED   = 5;

public:
	MyStrategy();

	void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& move);

	bool IsOilDanger() const;

	void simulateBreaking(double desiredSpeed, int &ticksToBrake, double &distanceToBrake) const;

private:
	struct Statistics
	{
		Statistics();

		double   m_maxSpeed;
		double   m_currentSpeed;
		double   m_previousSpeed;
		int      m_lastWallCollisionTick;
		int      m_lastCollisionEscapeTick;
		bool     m_isEscapingCollision;
		int      m_lastOilTick;
		uint64_t m_sumSpeed;

		double m_recentSpeeds[30];

		void output(int ticks)    const;

		double recentSpeed()      const { return std::accumulate(std::begin(m_recentSpeeds), std::end(m_recentSpeeds), 0.0) / recentSpeedTicks(); }
		int    recentSpeedTicks() const { return std::extent<decltype(m_recentSpeeds)>::value; }
	};

	static const double k_angleFactor;

	const model::Car*   m_self;
	const model::World* m_world;
	const model::Game*  m_game;
	const model::Move*  m_move;

	Statistics           m_statistics;
	std::unique_ptr<Map> m_map;

	DebugVisualizer      m_debug;

	void updateStates(const model::Car& self, const model::World& world, const model::Game& game, const model::Move& move);

	bool isWallCollision();
	bool isMovingForward()
	{
		bool isLeftDirection = std::cos(m_self->getAngle()) > 0;
		bool isUpDirection   = std::sin(m_self->getAngle()) > 0;

		bool isMovingLeft = m_self->getSpeedX() > 0;
		bool isMovindUp   = m_self->getSpeedY() > 0;

		return isMovindUp == isUpDirection || isMovingLeft == isLeftDirection;
	}

	void printFinishStats() const;
};

#endif
