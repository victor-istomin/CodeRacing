#include "MyStrategy.h"
#include "Utils.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <set>

using namespace model;
const double MyStrategy::k_angleFactor = 32.0 / PI;

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move)
{
	updateStates(self, world, game, move);
	DebugMessage debug = DebugMessage(m_debug, *m_map, self, world, game, move);

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
	Point nextWaypoint = m_map->getTileCenter(self.getNextWaypointX(), self.getNextWaypointY());
	double distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);

	bool isPassThruWaypoint = false; // TODO - fixme

	/* optimize cornering */
	const double FAR = game.getTrackTileSize() * 1.3;
	double cornerTileOffset = 0.25 * game.getTrackTileSize();
	int offsetDirection = distanceToWaypoint > FAR ? -1 : 1;

	TileType waypointTileType = m_map->getTileType(self.getNextWaypointX(), self.getNextWaypointY());
	switch (waypointTileType)
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
	case CROSSROADS:
		isPassThruWaypoint = true;
		break;

	case TOP_HEADED_T:
	{
		double margin = game.getTrackTileMargin() + game.getCarWidth() / 2.1;
		nextWaypoint = m_map->getTileCorner(self.getNextWaypointX(), self.getNextWaypointY());
		distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);

		if (distanceToWaypoint > game.getTrackTileSize() * 1.7)
		{
			nextWaypoint = nextWaypoint + Point(1.3 * m_game->getTrackTileSize(), m_game->getTrackTileSize() - margin);
		}
		else
		{ 
			nextWaypoint = nextWaypoint + Point(0.5 * m_game->getTrackTileSize(), 0.5 * m_game->getTrackTileSize());
		}
		break;
	}

	case RIGHT_HEADED_T:
	{
		double margin = game.getTrackTileMargin() + game.getCarWidth() / 2.1;
		nextWaypoint = m_map->getTileCorner(self.getNextWaypointX(), self.getNextWaypointY());
		distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);

		if (distanceToWaypoint > game.getTrackTileSize() * 1.7)
		{
			nextWaypoint = nextWaypoint + Point(margin, 1.4 * m_game->getTrackTileSize());
		}
		else
		{
			nextWaypoint = nextWaypoint + Point(m_game->getTrackTileSize() + margin, m_game->getTrackTileSize() - margin);
		}
		break;
	}

	default:
		break;
	}

	double angleToWaypoint = self.getAngleTo(nextWaypoint.x, nextWaypoint.y);
	distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);
	debug.m_destination = nextWaypoint;

	if (isWallCollision())
	{
		move.setEnginePower(-1);
		move.setWheelTurn(-2 * angleToWaypoint * k_angleFactor);
		move.setUseNitro(false);
		return; // TODO
	}

	// apply brake when changing rear/front gear
	bool wantsForward = self.getEnginePower() >= 0 || move.getEnginePower() > 0;
	bool isStopped = self.getSpeedX() == 0 && self.getSpeedY() == 0;
	if (!isStopped && (isMovingForward() != wantsForward))
	{
		move.setBrake(true);
	}

	int degreesToWaypoint = static_cast<int>(std::abs(angleToWaypoint) * PI / 180);
	double correctedDistanceToWaypoint = (isPassThruWaypoint ? 1.5 : 1.0) * distanceToWaypoint;   // TODO - fixme
	if (world.getTick() > game.getInitialFreezeDurationTicks() 
		&& degreesToWaypoint < 10 && correctedDistanceToWaypoint > 3 * game.getTrackTileSize()
		&& waypointTileType != TOP_HEADED_T
		&& waypointTileType != RIGHT_HEADED_T)
	{
		move.setUseNitro(true);
	}
	
	// move
	double turnDirection = isMovingForward() ? 1 /* front gear*/ : -1.5 /* read gear*/;
	move.setWheelTurn(turnDirection * angleToWaypoint * k_angleFactor);
	move.setEnginePower(1);

	bool isVeryCareful = self.getDurability() < 0.3 || waypointTileType == RIGHT_HEADED_T || waypointTileType == TOP_HEADED_T;
	static const double SAFE_SPEED = isVeryCareful ? 7 : 10;

	int ticksToBrake = 0;
	double distanceToBrake = 0;
	simulateBreaking(SAFE_SPEED, ticksToBrake, distanceToBrake);
	
	// brake before waypoint
	double distanceWithGap = distanceToBrake + game.getTrackTileSize() / (isVeryCareful ? 4 : 5);
	if (distanceWithGap > distanceToWaypoint && m_statistics.m_currentSpeed > SAFE_SPEED && !isPassThruWaypoint)
	{
		move.setBrake(true);
		move.setUseNitro(false);
	}

	// it's good idea to spill oil before apex 
	bool isJustBeforeTurn = !isPassThruWaypoint && distanceToWaypoint < game.getTrackTileSize() / 2;
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
	if (isMovingForward() && correctedDistanceToWaypoint > game.getTrackTileSize() * 2)
	{
		auto bonus = std::find_if(world.getBonuses().cbegin(), world.getBonuses().cend(), 
			[&self, &game, &nextWaypoint, distanceToWaypoint, angleToWaypoint](const Bonus& bonus)
		{
			double searchScope = 3 * PI / 180;
			double health = self.getDurability();
			double relativeDist = distanceToWaypoint - self.getDistanceTo(bonus);

			switch (bonus.getType())
			{
			case NITRO_BOOST:
			case OIL_CANISTER:
				searchScope *= 2;
				break;

			case REPAIR_KIT:
				searchScope *= (health > 0 && health < 0.9) ? std::min(3.0, 1 / health) : 0.5;
				relativeDist *= (health > 0 && health < 0.5) ? std::min(2.0, 1 / (health + 0.5)) : 1;
				break;

			default:
				break;
			}

			const double NEAR = game.getTrackTileSize() * 1.5;
			double angleToBonus = self.getAngleTo(bonus);
            bool isAcceptableAngle = self.getDistanceTo(bonus) < NEAR 
                                      ? std::abs(angleToBonus) < searchScope 
                                      : std::abs(angleToBonus - angleToWaypoint) < searchScope;			

			return isAcceptableAngle && relativeDist > game.getTrackTileSize();
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
	m_self = &self;
	m_world = &world;
	m_game = &game;
	m_move = &move;

	if (!m_map)
		m_map.reset(new Map(game, world));

	m_statistics.m_previousSpeed = m_statistics.m_currentSpeed;
	m_statistics.m_currentSpeed = std::hypot(self.getSpeedX(), self.getSpeedY());
	m_statistics.m_maxSpeed = std::max(m_statistics.m_maxSpeed, m_statistics.m_currentSpeed);

	m_statistics.m_sumSpeed += static_cast<uint64_t>(m_statistics.m_currentSpeed);

	if (self.getDurability() > 0 && world.getTick() > game.getInitialFreezeDurationTicks())
		m_statistics.m_recentSpeeds[world.getTick() % m_statistics.recentSpeedTicks()] = m_statistics.m_currentSpeed; 
}

bool MyStrategy::isWallCollision()
{
	if (m_self->isFinishedTrack())
		return false;

	static const double STOPPED   = 7;
	static const int    TICKS_GAP = 85;
	static const int    COLLIDE_INTERVAL = 160;
	static const int    COOLDOWN_TICKS = 20;
	static const double MIN_AVERAGE_SPEED = 1.5;

	int currentTick = m_world->getTick();
	int ticksSinceCollide = currentTick - m_statistics.m_lastWallCollisionTick;
	int ticksSinceEscape = currentTick - m_statistics.m_lastCollisionEscapeTick;
	int ticksEscaping = m_statistics.m_isEscapingCollision ? ticksSinceCollide : TICKS_GAP;
	bool isJustEscaped = ticksEscaping < TICKS_GAP;

	if (!m_statistics.m_isEscapingCollision)
	{
		bool isJustCollide =  currentTick < (m_game->getInitialFreezeDurationTicks() + m_statistics.recentSpeedTicks())
		                   || currentTick < (m_statistics.m_lastWallCollisionTick + m_statistics.recentSpeedTicks());

		bool isSlow = !isJustCollide && ticksSinceEscape > COLLIDE_INTERVAL && m_statistics.recentSpeed() < MIN_AVERAGE_SPEED;

		if (isMovingForward() && !isJustEscaped && ticksSinceCollide > COLLIDE_INTERVAL &&
			(   (m_statistics.m_previousSpeed > STOPPED && m_statistics.m_previousSpeed > m_statistics.m_currentSpeed * 4) 
		      || isSlow
			))
		{
			m_statistics.m_isEscapingCollision = true;
			m_statistics.m_lastWallCollisionTick = currentTick;
			return true;
		}
	}
	else // escaping from collision
	{ 
		if (ticksEscaping < COOLDOWN_TICKS)
		{
			// may be small rebound from wall, not really escaped, need cooldown
			return true;
		}
		else if (isJustEscaped || 
			(m_statistics.m_currentSpeed < STOPPED && ticksSinceCollide < COLLIDE_INTERVAL))
		{
			return true; // still escaping
		}
		else 
		{
			// TODO - this is workaround 
			std::fill(std::begin(m_statistics.m_recentSpeeds), std::end(m_statistics.m_recentSpeeds), m_statistics.m_currentSpeed * 3);

			m_statistics.m_lastCollisionEscapeTick = currentTick;
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

MyStrategy::Statistics::Statistics()
	: m_maxSpeed(0), m_currentSpeed(0), m_previousSpeed(0), m_lastWallCollisionTick(0), m_lastCollisionEscapeTick(0), m_lastOilTick(0), m_sumSpeed(0)
	, m_isEscapingCollision(false) 
{
	std::fill(std::begin(m_recentSpeeds), std::end(m_recentSpeeds), 30.0); // prevent false collision detecting on game start
}

void MyStrategy::Statistics::output(int ticks) const
{
#define STAT(name)                << "  " #name << ": " << name << "; " << std::endl
#define STAT_COMPUTE(name, value) << "  " name  << ": " << (value) << "; " << std::endl

	std::cout << "Stats: " << std::endl
		STAT(m_maxSpeed)
		STAT(m_currentSpeed)
		STAT(m_previousSpeed)
		STAT(m_lastWallCollisionTick)
		STAT(m_isEscapingCollision)
		STAT(m_lastOilTick)
		STAT_COMPUTE("avg speed", m_sumSpeed / (double)ticks);
}
