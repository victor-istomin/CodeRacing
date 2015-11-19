#pragma once

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "Utils.h"
#include <cmath>
#include <cstdint>
#include <memory>

class MyStrategy : public Strategy 
{
public:
	MyStrategy();

	void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& move);

	void simulateBreaking(double desiredSpeed, int &ticksToBrake, double &distanceToBrake);

private:
	struct Statistics
	{
		Statistics() : m_maxSpeed(0), m_currentSpeed(0), m_previousSpeed(0), m_lastEscapeTick(0), m_lastOilTick(0), m_sumSpeed(0), m_isEscapingCollision(false) {}

		double   m_maxSpeed;
		double   m_currentSpeed;
		double   m_previousSpeed;
		int      m_lastEscapeTick;
		bool     m_isEscapingCollision;
		int      m_lastOilTick;
		uint64_t m_sumSpeed;

		void output(int ticks) const;
	};

	static const double k_angleFactor;

	const model::Car*   m_self;
	const model::World* m_world;
	const model::Game*  m_game;
	const model::Move*  m_move;

	Statistics           m_statistics;
	std::unique_ptr<Map> m_map;

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
