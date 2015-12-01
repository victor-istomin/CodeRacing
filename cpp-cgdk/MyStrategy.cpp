#include "MyStrategy.h"
#include "Utils.h"
#include "Map.h"
#include "PathFinder.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES
const double PointD::k_epsilon = 0.001;


#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <numeric>
#include <set>
#include <type_traits>
#include <cassert>
#include <iostream>

using namespace model;
const double MyStrategy::k_angleFactor = 32.0 / PI;

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move)
{
	updateStates(self, world, game, move);
	PathFinder::Path turnsAhead = getTurnsToWaypoint();
	DebugMessage debug = DebugMessage(m_debug, *m_map, self, world, game, move, turnsAhead);

	shootEnemy(move);
	
	move.setEnginePower(1.0);

	// get next waypoint
	PathFinder::Path waypointPath = getTurnsToWaypoint();
	if (waypointPath.empty())
	{
		// safe failsave workaround
		assert(!waypointPath.empty());
		TileNode* tileNode = m_map->getTileNodePtr(m_map->getTileNodeIndex(self.getNextWaypointX(), self.getNextWaypointY()));
		TilePathNode node(*tileNode);
		node.m_turnRelative = RelativeTurn::TURN_CLOCKWISE;
	}

	PointD nextWaypoint = getTurnEntryPoint(waypointPath.front());

	double distanceToWaypoint = self.getDistanceTo(nextWaypoint.x, nextWaypoint.y);

	bool isPassThruWaypoint = waypointPath.empty() ? false : waypointPath.front().m_turnRelative == RelativeTurn::TURN_NONE;

	TileType waypointTileType = m_map->getTileType(self.getNextWaypointX(), self.getNextWaypointY());

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

	double degreesToWaypoint = static_cast<int>(std::abs(angleToWaypoint) * 180 / PI);
	double correctedDistanceToWaypoint = (isPassThruWaypoint ? 1.5 : 1.0) * distanceToWaypoint;   // TODO - fixme
	if (world.getTick() > game.getInitialFreezeDurationTicks() 
		&& degreesToWaypoint < 10.0 && correctedDistanceToWaypoint > 3 * game.getTrackTileSize()
		&& waypointTileType != TOP_HEADED_T
		&& waypointTileType != RIGHT_HEADED_T
	    && waypointTileType != LEFT_HEADED_T  
		&& waypointTileType != BOTTOM_HEADED_T
	   )
	{
		move.setUseNitro(true);
	}

	
	// move
	double turnDirection = isMovingForward() ? 1 /* front gear*/ : -1.5 /* read gear*/;
	move.setWheelTurn(turnDirection * angleToWaypoint * k_angleFactor);
	move.setEnginePower(1);

	bool isVeryCareful = self.getDurability() < 0.3 
		|| waypointTileType == RIGHT_HEADED_T || waypointTileType == TOP_HEADED_T || waypointTileType == LEFT_HEADED_T || waypointTileType == BOTTOM_HEADED_T;

	double corneringSpeed = isVeryCareful ? CORNERING_SPEED_CAREFUL : CORNERING_SPEED_REGULAR;
	if (IsOilDanger())
		corneringSpeed /= 2;

	int ticksToBrake = 0;
	double distanceToBrake = 0;
	simulateBreaking(corneringSpeed, ticksToBrake, distanceToBrake);
	
	// brake before waypoint
	double distanceWithGap = distanceToBrake + game.getTrackTileSize() / (isVeryCareful ? 4 : 5);
	if (distanceWithGap > distanceToWaypoint && m_statistics.m_currentSpeed > corneringSpeed && !isPassThruWaypoint)
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

void MyStrategy::shootEnemy(Move &move)
{
	// it's good idea to shoot an enemy...
	if (m_self->getProjectileCount() > 0)
	{
		auto carToShoot = std::find_if(m_world->getCars().cbegin(), m_world->getCars().cend(), [this](const Car& car)
		{
			const double scope = 1 * PI / 180;
			return !car.isTeammate()
				&& m_self->getDistanceTo(car) < m_game->getTrackTileSize() * 1.5
				&& std::abs(m_self->getAngleTo(car)) < scope;
		});

		if (carToShoot != m_world->getCars().end())
		{
			move.setThrowProjectile(true);
		}
	}
}

bool MyStrategy::IsOilDanger() const
{
	if (m_self->getRemainingOiledTicks() > 0)
		return true; // already oiled

	int    ticksToOiledSpeed = 0;
	double distanceToOiledSpeed = 0;
	simulateBreaking(CORNERING_SPEED_OILED, ticksToOiledSpeed, distanceToOiledSpeed);

	return m_world->getOilSlicks().cend() != std::find_if(m_world->getOilSlicks().cbegin(), m_world->getOilSlicks().cend(),
		[this, ticksToOiledSpeed, distanceToOiledSpeed](const OilSlick& slick)
	{
		double distanceToSlick = m_self->getDistanceTo(slick);

		double xTicksToSlick = (slick.getX() - m_self->getX()) / m_self->getSpeedX();
		double yTicksToSlick = (slick.getY() - m_self->getY()) / m_self->getSpeedY();
		bool   canIntersect = xTicksToSlick >= 0 && yTicksToSlick > 0;

		double safeDistance = m_game->getTrackTileSize();
		double safeTime = ticksToOiledSpeed * 1.3;

		bool isDangerous = canIntersect
			&& (std::max(xTicksToSlick, yTicksToSlick) < safeTime || distanceToSlick < safeDistance);

		return isDangerous;
	});
}

void MyStrategy::simulateBreaking(double desiredSpeed, int &ticksToBrake, double &distanceToBrake) const
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
	else
		m_map->update(game, world);

	if (!m_pathFinder)
		m_pathFinder.reset(new PathFinder(*m_map));

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

PathFinder::Path MyStrategy::getTurnsToWaypoint()
{
	const auto& waypoints = m_world->getWaypoints();
	int waypointIndex = m_self->getNextWaypointIndex();
	int furtherWaypointIndex = (waypointIndex + 1) % waypoints.size();

	PointD current = PointD(m_self->getX(), m_self->getY());
	PointD waypoint = m_map->getTileCenter(waypoints[waypointIndex][0], waypoints[waypointIndex][1]);
	PointD furtherWaypoint = m_map->getTileCenter(waypoints[furtherWaypointIndex][0], waypoints[furtherWaypointIndex][1]);

	double selfAngle = m_self->getAngle();
	if (selfAngle < 0)
		selfAngle += 2 * PI;

	struct AngleConstraints
	{
		AbsoluteDirection direction;
		double angle;

		bool operator==(AbsoluteDirection dir) const { return this->direction == dir; }
	};

	static const AngleConstraints directionsByAngle[] =
	{ 
		{AbsoluteDirection::RIGHT, 0},
		{AbsoluteDirection::DOWN,  PI/2},
		{AbsoluteDirection::LEFT,  PI},
		{AbsoluteDirection::UP,    3 * PI / 2}
	};

	static const size_t DIRECTIONS_COUNT = std::extent<decltype(directionsByAngle)>::value;

	AngleConstraints relativeAngles[DIRECTIONS_COUNT];
	std::transform(std::begin(directionsByAngle), std::end(directionsByAngle), std::begin(relativeAngles), [selfAngle](AngleConstraints constraints) 
	{
		constraints.angle = std::abs(selfAngle - constraints.angle);
		return constraints;
	});
	
	std::sort(std::begin(relativeAngles), std::end(relativeAngles), [](const AngleConstraints& a, const AngleConstraints& b) { return a.angle < b.angle; });
	
	auto currentDirection = relativeAngles[0].direction; // closest angle constraint
	size_t directionQuarter = std::find(std::begin(directionsByAngle), std::end(directionsByAngle), currentDirection) - std::begin(directionsByAngle);

	// calculate turns
	PathFinder::Path path        = m_pathFinder->getPath(current, waypoint);
	PathFinder::Path furtherPath = m_pathFinder->getPath(waypoint, furtherWaypoint);
	if (!furtherPath.empty())
		furtherPath.pop_front(); // don't duplicate 1st waypoint tile
	std::copy(std::begin(furtherPath), std::end(furtherPath), std::back_inserter(path));

	assert(!path.empty());
	if (path.empty())
		return path;

	// convert 'went from' directions to 'turn to' directions
	for (auto it = path.begin(); it != path.end(); ++it)
	{
		auto nextIt = it; ++nextIt;
		auto turnTo = nextIt == path.end() ? AbsoluteDirection::UNKNOWN/*no information*/ : nextIt->m_turnAbsoluteFrom;

		it->m_turnAbsolute = turnTo;
	}
	
	PathFinder::Path turns;
	TilePathNode previous = path.front();
	currentDirection = previous.m_turnAbsolute;

	for (const TilePathNode& nextTile : path)
	{
		if (nextTile.m_pos == previous.m_pos)
			continue;  // skip first tile

		// ignore waypoint if no turn
		if (nextTile.m_turnAbsolute != currentDirection /*turn*/)
		{
			turns.emplace_back(nextTile);

			auto clockwise    = directionsByAngle[(directionQuarter + 1) % DIRECTIONS_COUNT].direction;
			auto turnRelative = (nextTile.m_turnAbsolute == clockwise) ? RelativeTurn::TURN_CLOCKWISE : RelativeTurn::TURN_COUNTER_CLOCKWISE;

			// if waypoint and no turn
			// TODO - dead if?
			if (nextTile.m_isWaypoint && nextTile.m_turnAbsolute == currentDirection)
				turnRelative = RelativeTurn::TURN_NONE;

			turns.back().m_turnRelative = turnRelative;

			currentDirection = nextTile.m_turnAbsolute;
			directionQuarter = std::find(std::begin(directionsByAngle), std::end(directionsByAngle), currentDirection) - std::begin(directionsByAngle);
		}

		previous = nextTile;
	}

	return turns;
}

PointD MyStrategy::getTurnEntryPoint(const TilePathNode& turn) const
{
	static const double FAR_DISTANCE = m_game->getTrackTileSize() * 1.7;

	int x = turn.m_pos.x;
	int y = turn.m_pos.y;

	PointD turnCenter      = m_map->getTileCenter(x, y);
	PointI selfTileIndex   = m_map->getTileIndex(PointD(m_self->getX(), m_self->getY()));
	bool isHorizontalEnrty = selfTileIndex.y == y;
	bool isVerticalEntry   = selfTileIndex.x == x;
	bool isFarFrom = m_self->getDistanceTo(turnCenter.x, turnCenter.y) > FAR_DISTANCE;

	if (!isVerticalEntry && !isHorizontalEnrty)
	{
		assert(isVerticalEntry || isHorizontalEnrty); // something went wrong
		return turnCenter;
	}

	PointD entry = m_map->getTurnOuterCorner(x, y, turn);
	double towardsDisplacement = isFarFrom ? m_game->getTrackTileSize() * 1.3 : m_game->getTrackTileSize() / 2;

	if (isHorizontalEnrty)
	{
		double centerDisplacement  = 1.8 * (turnCenter.y - entry.y);  // TODO - avoid magic

		entry.x += (selfTileIndex.x > x ? 1 : -1) * towardsDisplacement;
		entry.y += isFarFrom ? 0 : centerDisplacement;
	}

	if (isVerticalEntry)
	{
		double centerDisplacement = 1.8  * (turnCenter.x - entry.x);  // TODO - avoid magic

		entry.y += (selfTileIndex.y > y ? 1 : -1) * towardsDisplacement;
		entry.x += isFarFrom ? 0 : centerDisplacement;
	}

	return entry;
}

MyStrategy::Statistics::Statistics()
	: m_maxSpeed(0), m_currentSpeed(0), m_previousSpeed(0), m_lastWallCollisionTick(0), m_lastCollisionEscapeTick(0), m_lastOilTick(0), m_sumSpeed(0)
	, m_isEscapingCollision(false) 
{
	std::fill(std::begin(m_recentSpeeds), std::end(m_recentSpeeds), 30.0); // prevent false collision detecting on game start
}

void MyStrategy::Statistics::output(int ticks) const
{
#ifdef _DEBUG
#  define STAT(name)                << "  " #name << ": " << name << "; " << std::endl
#  define STAT_COMPUTE(name, value) << "  " name  << ": " << (value) << "; " << std::endl

	std::cout << "Stats: " << std::endl
		STAT(m_maxSpeed)
		STAT(m_currentSpeed)
		STAT(m_previousSpeed)
		STAT(m_lastWallCollisionTick)
		STAT(m_isEscapingCollision)
		STAT(m_lastOilTick)
		STAT_COMPUTE("avg speed", m_sumSpeed / (double)ticks);
#endif
}
