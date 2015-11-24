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
	std::hash<int> intHash;

	typedef std::map<double/*cost*/, Node*> CostNodeMap;
	typedef std::unordered_set<Node*, decltype(nodeHasher), decltype(nodeEquals)> ClosedNodes;   // nodes array is fixed size. It's ok to work woth pointers here

	ClosedNodes closedSet = ClosedNodes(1, nodeHasher, nodeEquals);

	/*** test node-point conversion 
	size_t dbgIndex = map.getNodeIndex(10, 10);
	Node* dbgNode = map.getNodePtr(dbgIndex);
	Node* dbgNode2 = map.getNodePtr(10, 10);
	PointD dbgPoint = map.nodeToPoint(*dbgNode);
	Node* dbgNode3 = map.getNodePtr(dbgPoint);
	PointD dbgPoint2 = map.nodeToPoint(*dbgNode3);
	***/

	Node* start = map.getNodePtr(from);
	start->m_hx = map.getHeuristicsTo(*start, to);
	start->m_gx = 0;

	map.isNodePassable(*start);

	CostNodeMap openSet;
	openSet[start->fx()] = start;

	Path path;
	
	while (!openSet.empty())
	{
		Node* currentNode = openSet.begin()->second;

		closedSet.insert(currentNode);
		openSet.erase(openSet.begin());

		// reconstruct path if at goal position

		PointD currentNodePoint = map.nodeToPoint(*currentNode);
		if (map.isGoalPoint(currentNodePoint, to))
		{
			Node* previousNode = currentNode;
			while (previousNode != nullptr)
			{
				path.push_front(map.nodeToPoint(*previousNode));
				previousNode = previousNode->m_cachedParent;
			}

			return path;
		}

		// add neighbors to open list

		map.fillNeighbors(*currentNode, [&map, &currentNode, &to, &openSet, &closedSet](Node* candidate)
		{
			if (candidate == currentNode || candidate == currentNode->m_cachedParent)
				return false;

			double newGx = currentNode->m_gx + map.getTransitionCost(*currentNode, *candidate);
			double newHx = map.getHeuristicsTo(candidate, to); // TODO (!) - clarify Node/Point conversion!
			double newFx = newGx + newHx;
			//candidate->m_cachedParent = currentNode;


			/* dbg duplicate detection */
			dbgDuplicateCheck(candidate);

			bool isAlreadyClosed = closedSet.find(candidate) != closedSet.end();
			if (isAlreadyClosed && candidate->fx() <= newFx)
				return;

			std::cout << "dbg: opened (" << candidate->m_pos.x << ", " << candidate->m_pos.y << "); parent: " << std::hex << candidate->m_cachedParent << std::dec << std::endl;

			// update stats only if new path through close node is shorter

			candidate->m_gx = newGx;
			candidate->m_hx = newHx;
			candidate->m_cachedParent = currentNode;

			openSet[candidate->fx()] = candidate;
			return true;
		});
	}

	return path;
}

void PathFinder::dbgDuplicateCheck(Node* dbgPoint1)
{
	std::set<Node*> existing;
	while (dbgPoint1->m_cachedParent != nullptr)
	{
		if (existing.find(dbgPoint1->m_cachedParent) != existing.end())
			assert(0);

		existing.insert(dbgPoint1->m_cachedParent);
		dbgPoint1->m_cachedParent = dbgPoint1->m_cachedParent->m_cachedParent;
	}
}
