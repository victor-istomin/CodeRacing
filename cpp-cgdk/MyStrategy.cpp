#include "MyStrategy.h"
#include "Utils.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <set>

using namespace model;

const double MyStrategy::k_angleFactor = 32.0 / PI;


void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move)
{
	updateStates(self, world, game, move);

	// it's good idea to shoot an enemy...
	if (self.getProjectileCount() > 0)
	{
		auto carToShoot = std::find_if(world.getCars().cbegin(), world.getCars().cend(), [&self, &game](const Car& car)
		{
			const double scope = 1 * PI / 180;
			return !car.isTeammate()
				&& self.getDistanceTo(car) < game.getTrackTileSize() * 1.5
				&& std::abs(self.getAngleTo(car)) < scope;
		});

		if (carToShoot != world.getCars().end())
		{
			move.setThrowProjectile(true);
		}
	}
	
	move.setEnginePower(1.0);

	// get next waypoint
	Point nextWaypoint = Point::fromTileIndex(game, self.getNextWaypointX(), self.getNextWaypointY());
	double distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);
	/* optimize cornering */
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

	double angleToWaypoint = self.getAngleTo(nextWaypoint.x, nextWaypoint.y);
	distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);

	if (isWallCollision())
	{
		move.setEnginePower(-1);
		move.setWheelTurn(-2 * angleToWaypoint * k_angleFactor);
		return; // TODO
	}

	int degreesToWaypoint = static_cast<int>(std::abs(angleToWaypoint) * PI / 180);
	if (world.getTick() > game.getInitialFreezeDurationTicks() && ((degreesToWaypoint % 90) < 10)
		&& distanceToWaypoint > 3 * game.getTrackTileSize())
	{
		move.setUseNitro(true);
	}

	
	// move
	double turnDirection = isMovingForward() ? 1 /* front gear*/ : -1.5 /* read gear*/;  //TODO
	move.setWheelTurn(turnDirection * angleToWaypoint * k_angleFactor);
	move.setEnginePower(1);

	static const double SAFE_SPEED = 10;

	int ticksToBrake = 0;
	double distanceToBrake = 0;
	simulateBreaking(SAFE_SPEED, ticksToBrake, distanceToBrake);
	
	// brake before waypoint
	double distanceWithGap = distanceToBrake + game.getTrackTileSize() / 5;
	if (distanceWithGap > distanceToWaypoint && m_statistics.m_currentSpeed > SAFE_SPEED)
	{
		move.setBrake(true);
		move.setUseNitro(false);
	}

	// it's good idea to spill oil before apex 
	bool isJustBeforeTurn = distanceToWaypoint < game.getTrackTileSize() / 2;
	if (self.getOilCanisterCount() > 0 && isJustBeforeTurn)
	{
		const int OIL_SPILL_DELAY = 300;
		int tick = world.getTick();
		if (m_statistics.m_lastOilTick + OIL_SPILL_DELAY < tick)
		{
			move.setSpillOil(true);
			m_statistics.m_lastOilTick = tick;
		}
	}

	// may correct angle to pick up some interesting
	if (isMovingForward() && distanceToWaypoint > game.getTrackTileSize() * 2)
	{
		auto bonus = std::find_if(world.getBonuses().cbegin(), world.getBonuses().cend(), [&self, &game, &nextWaypoint](const Bonus& bonus)
		{
			double searchScope = 3 * PI / 180;
			double health = self.getDurability();
			switch (bonus.getType())
			{
			case NITRO_BOOST:
			case OIL_CANISTER:
				searchScope *= 2;
				break;

			case REPAIR_KIT:
				searchScope *= (health > 0 && health < 0.9) ? std::min(3.0, 1 / health) : 0.5;
				break;

			default:
				break;
			}

			const double relativeDist = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y) - self.getDistanceTo(bonus);
			const double relativeAngle = std::abs(self.getAngleTo(nextWaypoint.x, nextWaypoint.y) - self.getAngleTo(bonus));

			return std::abs(self.getAngleTo(bonus)) < searchScope
				&& relativeDist > game.getTrackTileSize();
		});

		if (bonus != world.getBonuses().cend())
		{
			move.setWheelTurn(self.getAngleTo(*bonus) * k_angleFactor);
		}
	}

	printFinishStats();
}


void MyStrategy::simulateBreaking(double desiredSpeed, int &ticksToBrake, double &distanceToBrake)
{
	static const int    ITERATIONS_PER_STEP = 10;
	static const double INCREMENT = 1.0 / ITERATIONS_PER_STEP;

	ticksToBrake = 0;
	distanceToBrake = 0;
	
	for (double speed = m_statistics.m_currentSpeed; speed > desiredSpeed; speed *= 1.0 - m_game->getCarMovementAirFrictionFactor())
	{
		for (int i = 0; i < ITERATIONS_PER_STEP; ++i)
		{
			speed -= m_game->getCarCrosswiseMovementFrictionFactor() * INCREMENT;
			distanceToBrake += speed * INCREMENT;
		}

		++ticksToBrake;
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

	m_statistics.m_sumSpeed += static_cast<uint64_t>(m_statistics.m_currentSpeed);

}

bool MyStrategy::isWallCollision()
{
	static const double STOPPED   = 6;
	static const int    TICKS_GAP = 85;

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

void MyStrategy::printFinishStats() const
{
#if defined _DEBUG

	if (!m_self->isFinishedTrack())
		return;

	static std::set<long long> finishedCars;

	if (finishedCars.find(m_self->getId()) == finishedCars.end())
	{
		finishedCars.insert(m_self->getId());

		std::cout << "Car " << m_self->getId() << " finished '" << m_world->getMapName() << "' at: " << m_world->getTick() << std::endl;
		m_statistics.output(m_world->getTick());
	}
#endif
}

void MyStrategy::Statistics::output(int ticks) const
{
#define STAT(name)                << "  " #name << ": " << name << "; " << std::endl
#define STAT_COMPUTE(name, value) << "  " name  << ": " << (value) << "; " << std::endl

	std::cout << "Stats: " << std::endl
		STAT(m_maxSpeed)
		STAT(m_currentSpeed)
		STAT(m_previousSpeed)
		STAT(m_lastEscapeTick)
		STAT(m_isEscapingCollision)
		STAT(m_lastOilTick)
		STAT_COMPUTE("avg speed", m_sumSpeed / (double)ticks);
}
