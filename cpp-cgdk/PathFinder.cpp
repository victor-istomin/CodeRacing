#include "PathFinder.h"
#include "Map.h"

#include <map>
#include <unordered_set>
#include <set>
#include <cassert>
#include <cmath>
#include <vector>

PathFinder::Path PathFinder::getPath(const PointD& from, const PointD& to)
{
	Map& map = m_map;
	map.resetPathFinderCaches();

	typedef std::multimap<double/*cost*/, TileNode*> CostNodeMap;
	typedef std::unordered_set<TileNode*> HashedNodes;   // nodes array is fixed size. It's ok to work with pointers here

	HashedNodes closedSet;

	TileNode* start  = map.getTileNodePtr(map.getTileNodeIndex(from));
	TileNode* finish = map.getTileNodePtr(map.getTileNodeIndex(to));

	CostNodeMap openSet;
	HashedNodes openSetHash;
	openSet.insert(std::make_pair(map.getHeuristicsTo(*start, *finish), start));
	openSetHash.insert(start);

	Path path;
	
	while (!openSet.empty())
	{
		TileNode* currentNode = openSet.begin()->second;

		closedSet.insert(currentNode);
		openSet.erase(openSet.begin());
		openSetHash.erase(currentNode);

		// reconstruct path if at goal position

		if (currentNode == finish)
		{
			TileNode* previousNode = currentNode;
			while (previousNode != nullptr)
			{
				path.emplace_front(*previousNode);
				previousNode = previousNode->m_transition.m_cachedParent;
			}

			return path;
		}

		// add neighbors to open list

		map.fillNeighbors(*currentNode, 
			[&map, &currentNode, &finish, &openSet, &openSetHash, &closedSet](TileNode* candidate, double currentGx)
		{
			if (candidate == currentNode || candidate == currentNode->m_transition.m_cachedParent)
				return;

			assert(candidate->m_type != model::EMPTY);

			TileNode::Transition newTransition = TileNode::Transition(*currentNode, *candidate);

			double newTransitionCost = newTransition.getCost(*candidate);
			double newGx = currentGx + newTransitionCost;
			double heuristics = map.getHeuristicsTo(*candidate, *finish); // TODO - can change or can not? This matters in closed node improvement
			double newFx = newGx + heuristics;

			bool isAlreadyOpened = openSetHash.find(candidate) != openSetHash.end();
			bool isAlreadyClosed = closedSet.find(candidate)   != closedSet.end();
			bool isAlreadySeen = isAlreadyOpened || isAlreadyClosed;
			double candidateOldGx = map.getCostFromStart(*candidate);

			if (isAlreadySeen && candidateOldGx <= newGx)
			{
				return;
			}
			else if (isAlreadySeen)
			{
				// purge old worst result
				double costKey = candidateOldGx + heuristics;
				CostNodeMap::iterator found = openSet.lower_bound(costKey);
				while (found != openSet.end() && found->first == costKey)  // TODO: epsilon?
				{
					CostNodeMap::iterator next = found;
					++next;

					if (found->second == candidate)
					{
						openSetHash.erase(found->second);
						openSet.erase(found);
					}

					found = next;
				}
			}

			// update stats only if new path through close node is shorter

			candidate->m_transition = newTransition;
			openSet.insert( std::make_pair(newFx, candidate) );
			openSetHash.insert(candidate);
			return;
		});
	}

	return path;
}

TilePathNode::TilePathNode(const TileNode& node)
	: m_pos(node.m_pos)
	, m_turnAbsolute(AbsoluteDirection::UNKNOWN)
	, m_turnAbsoluteFrom(node.m_transition.m_turnedDirection)
	, m_turnRelative(RelativeTurn::TURN_NONE)
	, m_isWaypoint(node.m_isWaypoint)
{
	// TODO - actualy, relative turn does not works
}
