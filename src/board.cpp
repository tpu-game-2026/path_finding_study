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

	std::multimap<float, Point> q;
	mass[始点.y][始点.x].visit(始点, mass[始点.y][始点.x]);
	q.insert({ Point::distance(始点,終点),始点 });//要素を追加

	
	while (!q.empty()) {
		//std::cout << q.size()<<'\n';

		Point nowPoint = q.begin()->second; //今回のマスに親を置く

		int distance = mass[nowPoint.y][nowPoint.x].getSteps();
		q.erase(q.begin());//先頭要素を削除
		mass[nowPoint.x][nowPoint.y].close(); //調べ終わったらクローズ

		//// 歩いた場所に印をつける(見やすさのために始点は書き換えない)

		// ★★★ todo: ここを素敵にしよう
		// 終点に向かって歩く

		//４方向を確認する
		for (int i = 0; i < 4; i++) {
			Point next = nowPoint;
			//各加算ごとに方向を変更
			next.x += (i == 0) ? 1 : (i == 1) ? -1 : 0;//next.xに(i==0)なら1を加算 (i==1)なら -1を加算 そうでないなら 0を加算
			next.y += (i == 2) ? 1 : (i == 3) ? -1 : 0;
			
			Mass& nextMass = mass[next.y][next.x];

			//移動できないなら次の方向へ
			if (!map_[next.y][next.x].canMove()) { continue; }

			//次のマスが探索済みなら次の方向へ
			if (nextMass.isClosed()) { continue; }

			int startstep = distance + nextMass.getCost();
			int oldstep = nextMass.getSteps();

			if (0 <= oldstep) {  //すでに訪れたことのあるマス。

				std::cout << oldstep << "<=" << startstep<< " "<< (oldstep <= startstep)<<'\n';
				if (oldstep <= startstep) { continue; }//最短かどうかの判別

				//古いキーを削除
				for (auto it = q.begin(); it != q.end(); ++it) {
					if (it->second == next) {
						q.erase(it);
						break;
					}
				}
			}

			//移動先として登録
			nextMass.visit(nowPoint, nextMass); //訪れたマスとして保存
			q.insert({ static_cast<float>(startstep) + Point::distance(next,終点),next });

			//候補地点がゴールの場合
			if (next == 終点) {
				
				int count=0;
				//親をたどって経路の復元
				Point p = next;
				while (p != 始点) {
					count++;
					p = mass[p.y][p.x].getParent();
					if (p != 始点) {
						mass[p.y][p.x].set(Mass::WAYPOINT);//通り道の場合は通り道表示にする。

						//std::cout << "ルート登録"<< p.y<<" " << p.x<<'\n';
					}
				}
				//std::cout << count;
				return true;
			}


		}

		



	}

	return true;
}
