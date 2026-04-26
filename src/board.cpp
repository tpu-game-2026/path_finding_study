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


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>>& mass) const
{
	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	// 経路探索
	std::multimap<float, Point> q;
	mass[始点.y][始点.x].visit(始点, mass[始点.y][始点.x], 0.0f);
	q.insert({ Point::distance(始点, 終点), 始点 });
	while (!q.empty()) {
		Point 現在 = q.begin()->second;
		float distance = mass[現在.y][現在.x].getSteps();
		q.erase(q.begin());
		mass[現在.y][現在.x].close();

		// 4方向に移動できるか
		for (int i = 0; i < 4; i++) {
			Point 次 = 現在;
			次.x += (i == 0) ? 1 : (i == 1) ? -1 : 0;
			次.y += (i == 2) ? 1 : (i == 3) ? -1 : 0;
			Mass& 次のマス = mass[次.y][次.x];
			if (!map_[次.y][次.x].canMove()) { continue; }
			if (次のマス.isClosed()) { continue; }
			float 始点からの歩数 = distance + 次のマス.getCost();
			float 以前の歩数 = 次のマス.getSteps();
			if (0 <= 以前の歩数) {
				if (以前の歩数 <= 始点からの歩数) { continue; }
				// 古いキーの削除
				auto range = q.equal_range(以前の歩数);
				for (auto it = q.begin(); it != q.end(); it++) {
					if (it->second == 次) { q.erase(it); break; }
				}
			}

			次のマス.visit(現在, mass[現在.y][現在.x], 次のマス.getCost());
			q.insert({ static_cast<float>(始点からの歩数) + Point::distance(次, 終点), 次 });

			if (次 == 終点)
			{
				// 終点に到達したら、親をたどって経路を復元する
				Point p = 現在;
				while (p != 始点) {
					if (p.x >= 0 && p.y >= 0)
					{
						mass[p.y][p.x].set(Mass::WAYPOINT);
					}
					p = mass[p.y][p.x].getParent();
				}
				return true;
			}
		}
	}

	// 経路探索
	//Point 現在 = 始点;
	//while (現在 != 終点) {
	//	// 歩いた場所に印をつける(見やすさのために始点は書き換えない)
	//	if (現在 != 始点){mass[現在.y][現在.x].set(Mass::WAYPOINT);}

	//	// ★★★ todo: ここを素敵にしよう
	//	// 終点に向かって歩く
	//	if (現在.x < 終点.x) { 現在.x++; continue; }
	//	if (終点.x < 現在.x) { 現在.x--; continue; }
	//	if (現在.y < 終点.y) { 現在.y++; continue; }
	//	if (終点.y < 現在.y) { 現在.y--; continue; }
	//}

	return true;
}
