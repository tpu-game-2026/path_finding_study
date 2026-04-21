#include "board.h"
#include <queue>
#include <tuple>
#include <limits>
#include <algorithm>
#include <cmath>

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
	Point 現在 = 始点;
	while (現在 != 終点) {
		// 歩いた場所に印をつける(見やすさのために始点は書き換えない)
		if (現在 != 始点){mass[現在.y][現在.x].set(Mass::WAYPOINT);}

		// 終点に向かって歩く（A* による最適経路探索）
		if (map_.empty()) return false;
		
		const int h = static_cast<int>(map_.size());
		const int w = static_cast<int>(map_[0].size());
		auto inBounds = [&](int x, int y) { return x >= 0 && x < w && y >= 0 && y < h; };
		
		// マップ上の最小ステップコストを計算
		double minStepCost = 0.3f; // ROAD の最小コスト
		
		// ヒューリスティック（マンハッタン距離）
		auto manhattan = [](const Point& a, const Point& b) {
			return std::abs(a.x - b.x) + std::abs(a.y - b.y);
		};
		
		// A* のデータ構造
		struct Node { double f; double g; int x; int y; };
		struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
		std::priority_queue<Node, std::vector<Node>, Cmp> open;
		
		const double INF = std::numeric_limits<double>::infinity();
		std::vector<std::vector<double>> gScore(h, std::vector<double>(w, INF));
		std::vector<std::vector<Point>> parent(h, std::vector<Point>(w, Point{-1,-1}));
		
		// 初期化
		gScore[現在.y][現在.x] = 0.0;
		double h0 = manhattan(現在, 終点) * minStepCost;
		open.push(Node{ h0, 0.0, 現在.x, 現在.y });
		
		const int dx[4] = { 1, -1, 0, 0 };
		const int dy[4] = { 0, 0, 1, -1 };
		
		bool found = false;
		while (!open.empty() && !found) {
			Node cur = open.top();
			open.pop();
			
			if (cur.g > gScore[cur.y][cur.x] + 1e-9) continue;
			
			if (cur.x == 終点.x && cur.y == 終点.y) {
				// ゴール到達、経路復元
				Point p{ cur.x, cur.y };
				while (!(p.x == 現在.x && p.y == 現在.y)) {
					Point 親 = parent[p.y][p.x];
					if (親.x == -1 && 親.y == -1) break;
					if (!(p.x == 終点.x && p.y == 終点.y)) {
						mass[p.y][p.x].set(Mass::WAYPOINT);
					}
					p = 親;
				}
				現在 = 終点;
				found = true;
				break;
			}
			
			for (int i = 0; i < 4; ++i) {
				int nx = cur.x + dx[i];
				int ny = cur.y + dy[i];
				if (!inBounds(nx, ny)) continue;
				
				if (!map_[ny][nx].canMove()) continue;
				
				double stepCost = static_cast<double>(map_[ny][nx].getCost());
				double tentative_g = cur.g + stepCost;
				
				if (tentative_g + 1e-9 < gScore[ny][nx]) {
					gScore[ny][nx] = tentative_g;
					parent[ny][nx] = Point{ cur.x, cur.y };
					double hval = manhattan(Point{nx, ny}, 終点) * minStepCost;
					double fval = tentative_g + hval;
					open.push(Node{ fval, tentative_g, nx, ny });
				}
			}
		}
		
		if (!found) return false;
	}

	return true;
}
