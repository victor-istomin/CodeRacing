#include "MyStrategy.h"
#include "Utils.h"
#include "Map.h"
#include "PathFinder.h"
#include "LineEquation.h"

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
	Path turnsAhead = getTurnsToWaypoint();
	DebugMessage debug = DebugMessage(m_debug, *m_map, self, world, game, move, turnsAhead);

	/*
	if (world.getTick() < 300)
		return;
	/**/


	shootEnemy(move);
	
	move.setEnginePower(1.0);

	// get next waypoint
	Path waypointPath = getTurnsToWaypoint();
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
	
	bool isVeryCareful = false;
	double corneringSpeed = calculateCorneringSpeed(self, waypointPath, waypointTileType, isVeryCareful);

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

double MyStrategy::calculateCorneringSpeed(const Car &self, const Path& waypointPath, TileType waypointTileType, bool& isVeryCareful) const
{
	isVeryCareful = self.getDurability() < 0.3
		|| waypointTileType == RIGHT_HEADED_T || waypointTileType == TOP_HEADED_T || waypointTileType == LEFT_HEADED_T || waypointTileType == BOTTOM_HEADED_T;

	static const int CORNERING_SPEED_CAREFUL = 7;
	static const int CORNERING_SPEED_REGULAR = 10;

	double corneringSpeed = isVeryCareful ? CORNERING_SPEED_CAREFUL : CORNERING_SPEED_REGULAR;
	if (IsOilDanger())
		corneringSpeed /= 2;


	static const int SAFE_TURN_DISTANCE    = 2;
	static const int SAFEST_TURN_DISTANCE  = 4;
	static const int NO_ACTUAL_TURN_MARKER = 6;
	
	int futureTurnDistance = SAFE_TURN_DISTANCE; 
	if (waypointPath.size() > 1)
	{
		const auto& thisTurn = waypointPath.front();
		const auto& futureWaypoint = *(++waypointPath.begin());

		futureTurnDistance = futureWaypoint.m_turnAbsolute != AbsoluteDirection::UNKNOWN 
			? static_cast<int>(thisTurn.m_pos.distanceTo(futureWaypoint.m_pos))           // distance in tiles, not it points
			: SAFE_TURN_DISTANCE;

		if (futureWaypoint.m_isZigZag && thisTurn.m_isZigZag)
			futureTurnDistance = SAFE_TURN_DISTANCE;

		static int FORWARD_TURNS_LOOKUP = 2; 
		PointD thisTurnPoint = getTurnEntryPoint(thisTurn);
		double maxTurnAngle = m_self->getAngleTo(thisTurnPoint.x, thisTurnPoint.y);

		if (thisTurn.m_isZigZag && futureWaypoint.m_isZigZag)
		{
			int turnsIndex = 0;
			auto itFuture = waypointPath.begin();
			while (turnsIndex++ < FORWARD_TURNS_LOOKUP && itFuture != waypointPath.end())
			{
				++itFuture;
				PointD turnPoint = getTurnEntryPoint(*itFuture);  // TODO - check carefully, entry point is neither horizontal nor vertical!

				maxTurnAngle = std::max(maxTurnAngle, std::abs(m_self->getAngleTo(turnPoint.x, turnPoint.y)));
			}

			static double NO_TURN_THRESHOLD = PI / 8; // 22.5 degrees
			if (maxTurnAngle < NO_TURN_THRESHOLD)
				futureTurnDistance = NO_ACTUAL_TURN_MARKER;
		}
	}


	// TODO - improve this after zigzag improvement
	// BUG (minor): issue with safest turn detection on 'default, map01, map02' maps

	static const double FAST_SPEED_FACTOR    = 1.25;
	static const double FASTEST_SPEED_FACTOR = 1.15;
	static const double NO_TURN_FACTOR       = 3;

	if (futureTurnDistance >= SAFE_TURN_DISTANCE)
		corneringSpeed *= FAST_SPEED_FACTOR;
	if (futureTurnDistance >= SAFEST_TURN_DISTANCE) // not 'else if'
		corneringSpeed *= FASTEST_SPEED_FACTOR;
	if (futureTurnDistance >= NO_ACTUAL_TURN_MARKER)
		corneringSpeed *= NO_TURN_FACTOR;

	return corneringSpeed;
}

void MyStrategy::shootEnemy(model::Move& move)
{

	if (m_self->getProjectileCount() > 0 && m_self->getRemainingProjectileCooldownTicks() == 0)
	{
		const PointD selfPoint = PointD(*m_self);
		const Vec2<double> selfSpeed = Vec2<double>(m_self->getSpeedX(), m_self->getSpeedY());
		const LineEquation selfTrajectory = LineEquation::fromDirectionVector(selfPoint, selfSpeed);

		const double projectileSpeedX = m_game->getWasherInitialSpeed() * std::cos(m_self->getAngle());
		const double projectileSpeedY = m_game->getWasherInitialSpeed() * std::sin(m_self->getAngle());

		auto carToShoot = std::find_if(m_world->getCars().cbegin(), m_world->getCars().cend(), 
			[this, &selfTrajectory, &selfPoint, &selfSpeed, projectileSpeedX, projectileSpeedY](const Car& car)
		{
			static const double MAX_SHOOT_DISTANCE   = m_game->getTrackTileSize() * 2;
			static const double MAX_LOOKUP_DISTANCE  = 2 * MAX_SHOOT_DISTANCE;
			static const double SHORT_SHOOT_DISTANCE = m_game->getTrackTileSize() * 1.5;

			if (car.isTeammate() || m_self->getDistanceTo(car) > MAX_LOOKUP_DISTANCE 
			 || car.getDurability() < 0.0001 || car.isFinishedTrack())
				return false;    // TODO - check to don't injure teammate

			if (m_self->getType() == model::BUGGY && m_self->getDistanceTo(car) < SHORT_SHOOT_DISTANCE
				&& std::abs(m_self->getAngleTo(car)) < (m_game->getSideWasherAngle() / 2))
			{
				return true; // force shoot from buggy on short distance
			}

			Vec2<double> carSpeed = Vec2<double>(car.getSpeedX(), car.getSpeedY());
			LineEquation unitTrajectory = LineEquation::fromDirectionVector(PointD(car), carSpeed);
			PointD intersection;
			if (!selfTrajectory.isIntersects(unitTrajectory, intersection) 
			  || intersection.distanceTo(selfPoint) > MAX_SHOOT_DISTANCE )
				return false;

			// todo
			double projectileAngle = m_self->getAngleTo(intersection.x, intersection.y);
			if (std::abs(projectileAngle) > (m_game->getSideWasherAngle() / 2))
				return false;  // ok?

			double carAngle = car.getAngle() + car.getAngleTo(intersection.x, intersection.y);
			double carArriveSpeed = car.getSpeedX() * std::cos(carAngle) + car.getSpeedY() * std::sin(carAngle);
			double carTicksToHit = std::hypot(intersection.x - car.getX(), intersection.y - car.getY()) / carArriveSpeed;

			double projectileArriveSpeed = m_game->getWasherInitialSpeed() * std::cos(projectileAngle);
			double projectileTicksToHit = std::hypot(intersection.x - selfPoint.x, intersection.y - selfPoint.y) / projectileArriveSpeed;

			double ticksEpsilon = std::max(5.0, std::hypot(car.getHeight(), car.getWidth()) / carArriveSpeed / 4);

			bool shouldHit = std::abs(projectileTicksToHit - carTicksToHit) < ticksEpsilon;
			double distance = intersection.distanceTo(selfPoint);
			double minDistance = m_self->getType() == model::JEEP ? 1.5 * std::max(car.getWidth(), car.getHeight()) : 0;
			if (shouldHit && m_self->getType() == model::JEEP && distance > minDistance)
			{
				// don't shoot from jeep over walls and don't shoot oneself with ricochet
				distance = std::max(m_game->getTrackTileSize(), distance);
				Path pathToIntersection = m_pathFinder->getPath(Vec2d(/*don't care*/), selfPoint, intersection);
				shouldHit = (pathToIntersection.size() - 1) * m_game->getTrackTileSize() < 2/*disgonal*/ * distance + m_game->getTrackTileSize();
			}

			return shouldHit;
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

	static const int CORNERING_SPEED_OILED = 5;
	simulateBreaking(CORNERING_SPEED_OILED, ticksToOiledSpeed, distanceToOiledSpeed);

	return m_world->getOilSlicks().cend() != std::find_if(m_world->getOilSlicks().cbegin(), m_world->getOilSlicks().cend(),
		[this, ticksToOiledSpeed, distanceToOiledSpeed](const OilSlick& slick)
	{
		double distanceToSlick = m_self->getDistanceTo(slick);

		// todo: single variable?
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
	static const int    TICKS_GAP = 90;
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

Path MyStrategy::getTurnsToWaypoint()
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

	// TODO - avoid unnecessary zigzag turns?

	// calculate turns
	Vec2d selfSpeed = Vec2d(m_self->getSpeedX(), m_self->getSpeedY());
	Path  path      = m_pathFinder->getPath(selfSpeed, current, waypoint);

	Vec2d furtherSpeed = path.size() > 3 ? Vec2d(/*don't care*/) : (selfSpeed / std::max(1U, path.size()));
	Path  furtherPath  = m_pathFinder->getPath(furtherSpeed, waypoint, furtherWaypoint);
	if (!furtherPath.empty())
		furtherPath.pop_front(); // don't duplicate 1st waypoint tile
	std::copy(std::begin(furtherPath), std::end(furtherPath), std::back_inserter(path));

	assert(!path.empty());
	if (path.empty())
		return path;

	// convert 'went from' directions to 'turn to' directions
	for (auto it = path.begin(); it != path.end(); ++it)
	{
		auto nextIt   = it; ++nextIt;
		auto turnTo   = nextIt == path.end() ? AbsoluteDirection::UNKNOWN/*no information*/ : nextIt->m_turnAbsoluteFrom;
		bool isZigzag = nextIt == path.end() ? false : nextIt->m_isZigZag;

		it->m_turnAbsolute = turnTo;
		it->m_isZigZag = isZigzag;
	}
	
	Path turns;
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
	// radius of circle around a*a square is a*sqr(2). For a = 1 it's 1.4
	//static const double FAR_DISTANCE      = m_game->getTrackTileSize() * 1.4;
	//static const double VERY_FAR_DISTANCE = m_game->getTrackTileSize() * 2;
	static const double MIN_ENTRY_FACTOR = 1.5;
	static const double MAX_ENTRY_FACTOR = 3;
	static const double TILE_SIZE = m_game->getTrackTileSize();
	static const double ENTRY_DISTANCE = MIN_ENTRY_FACTOR * TILE_SIZE;

	int x = turn.m_pos.x;
	int y = turn.m_pos.y;

	PointD turnCenter      = m_map->getTileCenter(x, y);
	PointI selfTileIndex   = m_map->getTileIndex(PointD(m_self->getX(), m_self->getY()));
	bool isHorizontalEnrty = selfTileIndex.y == y;
	bool isVerticalEntry   = selfTileIndex.x == x;

	double distanceToTurn = m_self->getDistanceTo(turnCenter.x, turnCenter.y);

	bool isFarFrom = distanceToTurn > ENTRY_DISTANCE;

	if (!isVerticalEntry && !isHorizontalEnrty)
	{
		//assert(isVerticalEntry || isHorizontalEnrty); // something went wrong
		return turnCenter;
	}

	PointD entry = m_map->getTurnOuterCorner(x, y, turn);
	double towardsDisplacement = TILE_SIZE / 2;
	if (isFarFrom)
	{
		// TODO check this code - it should guide car to outer radius of turn between 3 and 1.5 tiles to turn
		double tilesToEntry = distanceToTurn / TILE_SIZE;
		double gap = std::hypot(m_self->getWidth(), m_self->getHeight()) / 1.5 / TILE_SIZE;
		double multiplier = std::min(MAX_ENTRY_FACTOR, tilesToEntry) - gap;

		towardsDisplacement = multiplier * TILE_SIZE;
	}

	// TODO - add speed correction?

	if (isHorizontalEnrty)
	{
		double centerDisplacement  = 2 * (turnCenter.y - entry.y);  // TODO - avoid magic
		if (turn.m_isZigZag)
		{
			//centerDisplacement *= 0.80;  // less center displacement for zigzag
			//towardsDisplacement *= 0.85;
		}

		entry.x += (selfTileIndex.x > x ? 1 : -1) * towardsDisplacement;
		entry.y += isFarFrom ? 0 : centerDisplacement;
	}

	if (isVerticalEntry)
	{
		double centerDisplacement = 2  * (turnCenter.x - entry.x);  // TODO - avoid magic
		if (turn.m_isZigZag)
		{
			//centerDisplacement *= 0.80;  // less center displacement for zigzag
			//towardsDisplacement *= 0.85;
		}

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
