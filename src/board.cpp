#include "board.h"
using namespace std;
const float INF = 2e9;

static float heuristic(const Point& p, const Point& goal)
{
	return 0.3f * (abs(goal.x - p.x) + abs(goal.y - p.y));
}

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
	// 総探索回数
	int expandCount = 0;

	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	vector<vector<bool>> isclosed
	(
		mass.size(),
		vector<bool>(mass[0].size(), false)
	);

	vector<vector<float>> dist
	(
		mass.size(),
		vector<float>(mass[0].size(), -1)
	);
	dist[始点.y][始点.x] = 0.0f;

	vector<vector<Point>> parents
	(
		mass.size(),
		vector<Point>(mass[0].size(), {-1, -1})
	);

	multimap<float, Point> q;
	q.insert({ heuristic(始点, 終点), 始点 });

	// 経路探索
	while (!q.empty())
	{
		Point now = q.begin()->second;
		q.erase(q.begin());

		if (isclosed[now.y][now.x]) continue;

		float distance = dist[now.y][now.x];
		isclosed[now.y][now.x] = true;
		expandCount++;

		if (now == 終点) break;

		for (int i = 0; i < 4; i++)
		{
			Point next = now;
			next.x += (i == 0) ? 1 : (i == 1) ? -1 : 0;
			next.y += (i == 2) ? 1 : (i == 3) ? -1 : 0;
			Mass& nextmass = mass[next.y][next.x];
			if (!map_[next.y][next.x].canMove()) continue;
			if (isclosed[next.y][next.x]) continue;

			float stepsfromstart = distance + nextmass.getCost();
			float nextsteps = dist[next.y][next.x];

			if (0 <= nextsteps)
			{
				if (nextsteps <= stepsfromstart) continue;
				for (auto it = q.begin(); it != q.end(); ++it)
				{
					if (it->second == next)
					{
						q.erase(it);
						break;
					}
				}
			}

			dist[next.y][next.x] = stepsfromstart;
			parents[next.y][next.x] = now;
			q.insert({ stepsfromstart + heuristic(next, 終点), next });
		}
	}

	if (parents[終点.y][終点.x].x == -1 && parents[終点.y][終点.x].y == -1) return false;		

	Point 現在 = 終点;
	while (現在 != 始点)
	{
		現在 = parents[現在.y][現在.x];
		if (現在 != 始点)
		{
			mass[現在.y][現在.x].set(Mass::WAYPOINT);
		}
	}
	std::cout << "総探索回数（クローズリストのマス数）：" << expandCount << endl;
	return true;
}
