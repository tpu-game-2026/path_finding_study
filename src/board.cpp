#include "board.h"
#include <queue>

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


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	// 経路探索
	std::queue<Point> q;
	q.push(始点);
	while(!q.empty()) {
		Point 現在 = q.front(); q.pop();
		// 4方向に移動できるか
		for (int i = 0; i < 4; i++) {
			Point 次 = 現在;
			次.x += (i == 0) ? 1 : (i == 1) ? -1 : 0;
			次.y += (i == 2) ? 1 : (i == 3) ? -1 : 0;
			if (!map_[次.y][次.x].canMove()) { continue; }
			if (mass[次.y][次.x].isVisited()) { continue; }
			mass[次.y][次.x].visit(現在);
			q.push(次);

			if (次 == 終点)
			{
				// 終点に到達したら、親をたどって経路を復元する
				Point p = 次;
				while (p != 始点) {
					p = mass[p.y][p.x].getParent();
					if (p != 始点) { mass[p.y][p.x].set(Mass::WAYPOINT); }
				}
				return true;
			}
		}
	}

	return true;
}
