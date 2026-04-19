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


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>> &mass) const
{
	mass[始点.y][始点.x].set(Mass::START);
	mass[終点.y][終点.x].set(Mass::GOAL);

	// 経路探索
	std::multimap<int, Point> q;// multimapで優先度付きキューを実装
	mass[始点.y][始点.x].visit(始点, mass[始点.y][始点.x]);
	q.insert({ 0,始点 });
	while(!q.empty()) {
		int distance = q.begin()->first;
		Point 現在 = q.begin()->second;
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
			int 始点からの歩数 = distance + 1;
			int 以前の歩数 = 次のマス.getSteps();
			if (0 <= 以前の歩数) {// 既に訪れた
				if (以前の歩数 <= 始点からの歩数) { continue; }// 以前の方が近い
				// 古いキーの削除
				auto range = q.equal_range(以前の歩数);
				for(auto it = range.first; it != range.second; ++it) {
					if (it->second == 次) { q.erase(it); break; }
				}
			}
			
			次のマス.visit(現在, 次のマス);
			q.insert({ 始点からの歩数, 次 });

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
