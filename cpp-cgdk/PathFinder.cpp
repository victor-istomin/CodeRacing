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

	auto nodeHasher = [&map, &intHash](const Node* n)  { return intHash(map.nodeToIndex(*n)); };
	auto nodeEquals = [](const Node* a, const Node* b) { return *a == *b; };

	typedef std::map<double/*cost*/, Node*> CostNodeMap;
	typedef std::unordered_set<Node*, decltype(nodeHasher), decltype(nodeEquals)> ClosedNodes;

	ClosedNodes closedSet = ClosedNodes(1, nodeHasher, nodeEquals);

	size_t dbgIndex = map.getNodeIndex(10, 10);
	Node* dbgNode = map.getNodePtr(dbgIndex);
	Node* dbgNode2 = map.getNodePtr(10, 10);
	PointD dbgPoint = map.nodeToPoint(*dbgNode);
	Node* dbgNode3 = map.getNodePtr(dbgPoint);
	PointD dbgPoint2 = map.nodeToPoint(*dbgNode3);


	Node* start = map.getNodePtr(from);
	PointD dbg = map.nodeToPoint(*start);

	start->m_hx = map.getHeuristicsTo(*start, to);
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

		map.fillNeighbors(*currentNode, [&map, &currentNode, &to, &openSet, &closedSet](Node& next)
		{
			if (&next == currentNode || &next == currentNode->m_cachedParent)
				return false;

			next.m_gx           = currentNode->m_gx + map.getTransitionCost(*currentNode, next);
			next.m_hx           = map.getHeuristicsTo(next, to);
			next.m_cachedParent = currentNode;

			std::cout << "dbg: opened (" << next.m_pos.x << ", " << next.m_pos.y << "); parent: " << std::hex << next.m_cachedParent << std::dec << std::endl;

			/* dbg duplicate detection */
			dbgDuplicateCheck(&next);

			ClosedNodes::iterator found = closedSet.find(&next);
			if (found != closedSet.end()) // incorrect pointers logic?
			{
				Node* previousResult = *found;
				if (previousResult->fx() > next.fx())
				{
					// improvement, forget old result
					*previousResult = next;
					openSet[next.fx()] = previousResult;
				}

				dbgDuplicateCheck(&next);
				return false; // already added
			}

			openSet[next.fx()] = &next;
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
