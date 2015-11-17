#include "MyStrategy.h"
#include "Utils.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>

using namespace model;

const double MyStrategy::k_angleFactor = 32.0;


void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move)
{
	updateStates(self, world, game, move);

    move.setEnginePower(1.0);
    move.setThrowProjectile(true);
    move.setSpillOil(true);

    if (world.getTick() > game.getInitialFreezeDurationTicks()) 
	{
        //move.setUseNitro(true);
    }

	// get next waypoint
	Point nextWaypoint = Point::fromTileIndex(game, self.getNextWaypointX(), self.getNextWaypointY());
	double angleToWaypoint = self.getAngleTo(nextWaypoint.x, nextWaypoint.y);

	if (isWallCollision())
	{
		move.setEnginePower(-1);
		move.setWheelTurn(-1 * angleToWaypoint * k_angleFactor / PI);
		return; // TODO
	}


	// move
	int turnDirection = isMovingForward() ? 1 /* front gear*/ : -1 /* read gear*/;
	move.setWheelTurn(turnDirection * angleToWaypoint * k_angleFactor / PI);
	move.setEnginePower(1);
}


MyStrategy::MyStrategy() 
	: m_self(nullptr)
	, m_world(nullptr)
	, m_game(nullptr)
	, m_move(nullptr)
{
}

void MyStrategy::updateStates(const model::Car& self, const model::World& world, const model::Game& game, const model::Move& move)
{
	m_self = &self;
	m_world = &world;
	m_game = &game;
	m_move = &move;

	m_statistics.m_previousSpeed = m_statistics.m_currentSpeed;
	m_statistics.m_currentSpeed = std::hypot(self.getSpeedX(), self.getSpeedY());
	m_statistics.m_maxSpeed = std::max(m_statistics.m_maxSpeed, m_statistics.m_currentSpeed);
}

bool MyStrategy::isWallCollision()
{
	static const double STOPPED = 5;
	static const int    TICKS_GAP = 10;

	int currentTick = m_world->getLastTickIndex();
	bool isNotJustEscaped = (currentTick - m_statistics.m_lastEscapeTick) > TICKS_GAP;

	if (!m_statistics.m_isEscapingCollision)
	{
		if (m_statistics.m_previousSpeed > m_statistics.m_currentSpeed * 3)
		{
			m_statistics.m_isEscapingCollision = true;
			m_statistics.m_lastEscapeTick = currentTick;
			return true;
		}
	}
	else // escaping from collision
	{ 
		if (m_statistics.m_currentSpeed < STOPPED)
		{
			m_statistics.m_lastEscapeTick = currentTick;
			return true; // still escaping
		}
		else
		{
			m_statistics.m_isEscapingCollision = false;
			m_statistics.m_previousSpeed = STOPPED;
			return false;
		}
	}

	return false;
}
