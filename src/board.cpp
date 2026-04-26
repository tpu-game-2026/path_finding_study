#include "board.h"

std::map<Mass::status, MassInfo> Mass::statusData =
{
	{ BLANK, { 1.0f, ' '}},
	{ WALL,  {-1.0f, '#'}},
	{ WATER, { 3.0f, '~'}},
	{ ROAD,  { 0.3f, '$'}},

	// 動的な要素
	{ START,	{-1.0f, 'S'}},
	{ GOAL,		{-1.0f, 'G'}},
	{ WAYPOINT, {-1.0f, 'o'}},

	{ INVALID,  {-1.0f, '\0'}},
};


bool Board::find(const Point& startPos, const Point& endPos, std::vector<std::vector<Mass>> &mass) const
{
	mass[startPos.y][startPos.x].set(Mass::START);
	mass[endPos.y][endPos.x].set(Mass::GOAL);

	// 経路探索
	std::multimap<float, Point> queue;
	queue.insert({Point::distance(startPos, endPos),startPos});

	while (!queue.empty())
	{
		Point currentPos = queue.begin()->second;
		int distance = mass[currentPos.y][currentPos.x].getSteps();
		queue.erase(queue.begin());
		mass[currentPos.y][currentPos.x].close();

		for (int i = 0; i < 4; i++)
		{
			Point nextPos = currentPos;
			nextPos.x += (i == 0) ? 1 : (i == 1) ? -1 : 0;
			nextPos.y += (i == 2) ? 1 : (i == 3) ? -1 : 0;

			Mass& nextMass = mass[nextPos.y][nextPos.x];
			if (!map_[nextPos.y][nextPos.x].canMove()) { continue; }
			if (nextMass.isClosed()) { continue; }

			int stepsFromStart = distance + nextMass.getCost();
			int stepsPrev = nextMass.getSteps();

			if (0 <= stepsPrev)
			{
				if (stepsPrev <= stepsFromStart) { continue; }

				for(auto it = queue.begin(); it != queue.end(); it++) {
					if (it->second == nextPos) {
						queue.erase(it);
						break;
					}
				}
			}

			nextMass.visit(currentPos, nextMass);
			queue.insert({ static_cast<float>(stepsFromStart) + Point::distance(nextPos, endPos), nextPos });

			if (nextPos == endPos)
			{
				Point p = nextPos;
				while (p != startPos)
				{
					p = mass[p.y][p.x].getParent();
					if (p != startPos) { mass[p.y][p.x].set(Mass::WAYPOINT); }
				}
				return true;
			}
		}
	}

	return true;
}
