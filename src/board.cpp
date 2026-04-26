#include "board.h"
#include <queue>
#include <limits>
#include <vector>
#include <algorithm>

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
    struct Node {
        Point pos;
        float g;
        float f;

        bool operator<(const Node& rhs) const {
            return f > rhs.f;
        }
    };

    int 高さ = (int)mass.size();
    int 幅 = (int)mass[0].size();

    std::priority_queue<Node> open;

    std::vector<std::vector<float>> cost(
        高さ, std::vector<float>(幅, std::numeric_limits<float>::max())
    );

    std::vector<std::vector<Point>> parent(
        高さ, std::vector<Point>(幅, { -1,-1 })
    );

    Point dir[4] = {
        {1,0},{-1,0},{0,1},{0,-1}
    };

    cost[始点.y][始点.x] = 0.0f;

    open.push({
        始点,
        0.0f,
        Point::distance(始点, 終点)
        });

    while (!open.empty())
    {
        Node now = open.top();
        open.pop();

        if (now.pos == 終点)
        {
            Point p = 終点;

            while (p != 始点)
            {
                if (p != 終点)
                    mass[p.y][p.x].set(Mass::WAYPOINT);

                p = parent[p.y][p.x];
            }

            mass[始点.y][始点.x].set(Mass::START);
            mass[終点.y][終点.x].set(Mass::GOAL);
            return true;
        }

        for (int i = 0; i < 4; i++)
        {
            Point next = now.pos + dir[i];

            if (next.x < 0 || next.x >= 幅 ||
                next.y < 0 || next.y >= 高さ)
                continue;

            if (!mass[next.y][next.x].canMove())
                continue;

            float newCost =
                cost[now.pos.y][now.pos.x] +
                mass[next.y][next.x].getCost();

            if (newCost < cost[next.y][next.x])
            {
                cost[next.y][next.x] = newCost;
                parent[next.y][next.x] = now.pos;

                float h =
                    Point::distance(next, 終点);

                open.push({
                    next,
                    newCost,
                    newCost + h
                    });
            }
        }
    }

	return true;
}
