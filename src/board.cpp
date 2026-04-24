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


bool Board::find(const Point& start, const Point& end, std::vector<std::vector<Mass>>& mass) const
{
	mass[start.y][start.x].set(Mass::START);
	mass[end.y][end.x].set(Mass::GOAL);

	// 経路探索
	std::multimap<float, Point> queue;	//優先度付きキュー
	mass[start.y][start.x].visit(start, mass[start.y][start.x]);
	queue.insert({ Point::distance(start,end),start });
	while (!queue.empty())
	{
		Point now = queue.begin()->second;
		int distance = mass[now.y][now.x].getSteps();
		queue.erase(queue.begin());
		mass[now.y][now.x].close();

		// ★★★ todo: ここを素敵にしよう
		for (int i = 0; i < 4; i++)
		{
			Point next = now;
			next.x += (i == 0) ? 1 : (i == 1) ? -1 : 0;
			next.y += (i == 2) ? 1 : (i == 3) ? -1 : 0;
			Mass& nextMass = mass[next.y][next.x];
			if (!map_[next.y][next.x].canMove()) { continue; }
			if (nextMass.isClosed()) { continue; }
			int stepsFromStart = distance + nextMass.getCost();
			int stepsOld = nextMass.getSteps();
			if (0 <= stepsOld)
			{
				if (stepsOld <= stepsFromStart) { continue; }

				auto range = queue.equal_range(stepsOld);
				for (auto it = queue.begin(); it != queue.end(); ++it)
				{
					if (it->second == next)
					{
						queue.erase(it);
						break;
					}
				}
			}
			nextMass.visit(now, nextMass);
			queue.insert({ static_cast<float>(stepsFromStart) + Point::distance(next,end),next });

			if (next == end)
			{
				Point p = next;
				while (p != start)
				{
					p = mass[p.y][p.x].getParent();
					if (p != start)
					{
						mass[p.y][p.x].set(Mass::WAYPOINT);
					}
				}
			}
		}
	}

	return true;
}
