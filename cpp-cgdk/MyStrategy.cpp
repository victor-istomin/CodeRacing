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

	// oil usage
	if(self.getOilCanisterCount() > 0)
	{
		const int OIL_SPILL_DELAY = 500;
		int tick = world.getTick();
		if (m_statistics.m_lastOilTick + OIL_SPILL_DELAY < tick)
		{
			move.setSpillOil(true);
			m_statistics.m_lastOilTick = tick;
		}
	}

	/*
	static bool dbgIsCollisionTest = false;
	if (!dbgIsCollisionTest)
	{
		if (!isWallCollision())
		{
			move.setEnginePower(0.5);
			return;
		}
		dbgIsCollisionTest = true;
	}*/

	// get next waypoint
	Point nextWaypoint = Point::fromTileIndex(game, self.getNextWaypointX(), self.getNextWaypointY());
	double distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);
	/* optimize cornering */
	{
		const double FAR = game.getTrackTileSize() * 1.3;
		double cornerTileOffset = 0.25 * game.getTrackTileSize();
		int offsetDirection = distanceToWaypoint > FAR ? -1 : 1;
		TileType tileType = world.getTilesXY()[self.getNextWaypointX()][self.getNextWaypointY()];
		switch (tileType) 
		{
		case LEFT_TOP_CORNER:
			nextWaypoint.x += cornerTileOffset * offsetDirection;
			nextWaypoint.y += cornerTileOffset * offsetDirection;
			break;
		case RIGHT_TOP_CORNER:
			nextWaypoint.x -= cornerTileOffset * offsetDirection;
			nextWaypoint.y += cornerTileOffset * offsetDirection;
			break;
		case LEFT_BOTTOM_CORNER:
			nextWaypoint.x += cornerTileOffset * offsetDirection;
			nextWaypoint.y -= cornerTileOffset * offsetDirection;
			break;
		case RIGHT_BOTTOM_CORNER:
			nextWaypoint.x -= cornerTileOffset * offsetDirection;
			nextWaypoint.y -= cornerTileOffset * offsetDirection;
			break;
		default:
			break;
		}
	}

	double angleToWaypoint = self.getAngleTo(nextWaypoint.x, nextWaypoint.y);
	distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);


	if (isWallCollision())
	{
		move.setEnginePower(-1);
		move.setWheelTurn(-2 * angleToWaypoint * k_angleFactor / PI);
		return; // TODO
	}

	int degreesToWaypoint = static_cast<int>(std::abs(angleToWaypoint) * PI / 180);
	if (world.getTick() > game.getInitialFreezeDurationTicks() && ((degreesToWaypoint % 90) < 10)
		&& distanceToWaypoint > 3 * game.getTrackTileSize())
	{
		move.setUseNitro(true);
	}

	
	// move
	int turnDirection = isMovingForward() ? 1 /* front gear*/ : -1.5 /* read gear*/;
	move.setWheelTurn(turnDirection * angleToWaypoint * k_angleFactor / PI);
	move.setEnginePower(1);


	int  speedModifier = m_statistics.m_currentSpeed > 30 ? 2 : 1;
	bool isNearWaypoint = distanceToWaypoint < game.getTrackTileSize() * speedModifier && m_statistics.m_currentSpeed > 15;
	bool isBigAngleToWaypoint = m_statistics.m_currentSpeed * std::abs(angleToWaypoint) > (10 * PI / 4);
	
	// brake before waypoint
	if (isBigAngleToWaypoint || isNearWaypoint)
	{
		move.setBrake(true);
		move.setUseNitro(false);
	}
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
	bool isFirstTimeInit = m_world == nullptr;

	m_self = &self;
	m_world = &world;
	m_game = &game;
	m_move = &move;

	m_statistics.m_previousSpeed = m_statistics.m_currentSpeed;
	m_statistics.m_currentSpeed = std::hypot(self.getSpeedX(), self.getSpeedY());
	m_statistics.m_maxSpeed = std::max(m_statistics.m_maxSpeed, m_statistics.m_currentSpeed);

	if (isFirstTimeInit)
	{
		m_statistics.m_lastOilTick = game.getInitialFreezeDurationTicks();
	}
}

bool MyStrategy::isWallCollision()
{
	static const double STOPPED   = 5;
	static const int    TICKS_GAP = 80;

	int currentTick = m_world->getTick();
	bool isJustEscaped = (currentTick - m_statistics.m_lastEscapeTick) < TICKS_GAP;

	if (!m_statistics.m_isEscapingCollision)
	{
		if (m_statistics.m_previousSpeed > m_statistics.m_currentSpeed * 3 && isMovingForward() && !isJustEscaped)
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
			return isJustEscaped; // still escaping
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
