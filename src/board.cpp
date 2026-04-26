#include "board.h"

std::map<Mass::status, MassInfo> Mass::statusData =
{
	{ BLANK, { 1.0f, ' '}},
	{ WALL,  {-1.0f, '#'}},
	{ WATER, { 3.0f, '~'}},
	{ ROAD,  { 0.3f, '$'}},

	// 動的な要素
	{ START,	{1.0f, 'S'}},
	{ GOAL,		{1.0f, 'G'}},
	{ WAYPOINT, {-1.0f, 'o'}},

	{ INVALID,  {-1.0f, '\0'}},
};


bool Board::find(const Point& start_Point, const Point& end_Point, std::vector<std::vector<Mass>> &mass) const
{
	mass[start_Point.y][start_Point.x].set(Mass::START);
	mass[end_Point.y][end_Point.x].set(Mass::GOAL);

	// 経路探索
	std::multimap<float, Point>queue; // 優先度付きキュー
	mass[start_Point.y][start_Point.x].visit(start_Point, mass[start_Point.y][start_Point.x]);
	queue.insert({ Point::distance(start_Point, end_Point),start_Point });
	while (!queue.empty()) {
		Point now = queue.begin()->second;
		int distance = mass[now.y][now.x].getSteps();
		queue.erase(queue.begin());
		// 既に処理済であればスキップ
		if (mass[now.y][now.x].isClosed()) continue;
		mass[now.y][now.x].close();

		for (int i = 0; i < 4; i++) {
			Point next = now;
			next.x += (i == 0) ? 1 : (i == 1) ? -1 : 0;
			next.y += (i == 2) ? 1 : (i == 3) ? -1 : 0;

			Mass& next_Mass = mass[next.y][next.x];

			if (!map_[next.y][next.x].canMove()) { continue; }
			if (next_Mass.isClosed()) { continue; }

			float newCost = distance + next_Mass.getCost();
			float previous_Steps = next_Mass.getSteps();

			if (previous_Steps < 0 || newCost < previous_Steps) {
				next_Mass.visit(now, next_Mass);
				float f = newCost + Point::distance(next, end_Point);
				queue.insert({ f,next });
			}

			if (next == end_Point) {
				// end_Pointに到達したら、親をたどって経路を復元
				Point p = next;
				while (p != start_Point) {
					p = mass[p.y][p.x].getParent();
					if (p != start_Point) { mass[p.y][p.x].set(Mass::WAYPOINT); }
				}
				return true;
			}
		}
	}

	return true;
}
