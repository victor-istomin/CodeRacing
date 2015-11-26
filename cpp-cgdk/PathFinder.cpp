#include "PathFinder.h"
#include "Map.h"

#include <map>
#include <unordered_set>
#include <set>
#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>

PathFinder::Path PathFinder::getPath(const PointD& from, const PointD& to)
{
	Map& map = m_map;

	typedef std::multimap<double/*cost*/, Node*> CostNodeMap;
	typedef std::unordered_set<Node*> HashedNodes;   // nodes array is fixed size. It's ok to work woth pointers here

	HashedNodes closedSet;

	/*** test node-point conversion 
	size_t dbgIndex = map.getNodeIndex(10, 10);
	Node* dbgNode = map.getNodePtr(dbgIndex);
	Node* dbgNode2 = map.getNodePtr(10, 10);
	PointD dbgPoint = map.nodeToPoint(*dbgNode);
	Node* dbgNode3 = map.getNodePtr(dbgPoint);
	PointD dbgPoint2 = map.nodeToPoint(*dbgNode3);
	***/

	Node* start = map.getNodePtr(from);
	Node* finish = map.getNodePtr(to);
	start->m_hx = map.getHeuristicsTo(*start, *finish);
	finish->m_hx = 0;

	map.isNodePassable(*start); // not needed?

	assert(start->m_isPassable);
	assert(finish->m_isPassable);

	CostNodeMap openSet;
	HashedNodes openSetHash;
	openSet.insert( std::make_pair(start->m_hx, start) );
	openSetHash.insert(start);

	Path path;
	
	while (!openSet.empty())
	{
		Node* currentNode = openSet.begin()->second;

		closedSet.insert(currentNode);
		openSet.erase(openSet.begin());
		openSetHash.erase(currentNode);

		// reconstruct path if at goal position

		PointD currentNodePoint = map.nodeToPoint(*currentNode);
		if (map.isGoalPoint(currentNodePoint, to))
		{
			Node* previousNode = currentNode;
			while (previousNode != nullptr)
			{
				/*** dbg ***/
				double gx = map.getCostFromStart(*previousNode);
				/*** dbg ***/
				path.push_front(map.nodeToPoint(*previousNode));
				previousNode = previousNode->m_cachedParent;
			}

			return path;
		}

		// add neighbors to open list

		map.fillNeighbors(*currentNode, 
			[&map, &currentNode, &finish, &openSet, &openSetHash, &closedSet](Node* candidate, double currentGx)
		{
			if (candidate == currentNode || candidate == currentNode->m_cachedParent)
				return;

			double newTransitionCost = map.getTransitionCost(*currentNode, *candidate);
			double newGx = currentGx + newTransitionCost;
			double newHx = map.getHeuristicsTo(*candidate, *finish); // TODO (!) - clarify Node/Point conversion!
			double newFx = newGx + newHx;

			bool isAlreadyOpened = openSetHash.find(candidate) != openSetHash.end();
			bool isAlreadyClosed = closedSet.find(candidate)   != closedSet.end();
			bool isAlreadySeen = isAlreadyOpened || isAlreadyClosed;
			double candidateOldGx = map.getCostFromStart(*candidate);
			if (isAlreadySeen && candidateOldGx <= newGx)
			{
				return;
			}
			else if (isAlreadyClosed)
			{
				double costKey = candidateOldGx + candidate->m_hx;
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

			candidate->m_hx                   = newHx;
			candidate->m_cachedTransitionCost = newTransitionCost;
			candidate->m_cachedParent         = currentNode;			

			openSet.insert( std::make_pair(newFx, candidate) );
			openSetHash.insert(candidate);
			return;
		});
	}

	return path;
}

