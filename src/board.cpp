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


bool Board::find(const Point& 始点, const Point& 終点, std::vector<std::vector<Mass>>& mass) const
{
	struct Node
	{
		Point pos;
		float g, f;
	};

	int height = (int)mass.size();
	int width = (int)mass[0].size();

	std::vector<std::vector<float>> gScore(height, std::vector<float>(width, 9999.0f));
	std::vector<std::vector<Point>> parentMap(height, std::vector<Point>(width, { -1,-1 }));
	std::vector<Node> openList;

	gScore[始点.y][始点.x] = 0.0f;
	float hStart = (float)(std::abs(終点.x - 始点.x) + std::abs(終点.y - 始点.y));
	openList.push_back({ 始点, 0.0f, hStart });

	while (!openList.empty())
	{
		auto it = std::min_element(openList.begin(), openList.end(), [](const Node& a, const Node& b)
			{
				return a.f < b.f;
			});

		Node current = *it;
		openList.erase(it);

		if (current.pos.x == 終点.x && current.pos.y == 終点.y)
		{
			Point p = parentMap[終点.y][終点.x];
			while (p.x != -1 && !(p.x == 始点.x && p.y == 始点.y)) 
			{
				mass[p.y][p.x].set(Mass::WAYPOINT);
				p = parentMap[p.y][p.x];
			}
			mass[始点.y][始点.x].set(Mass::START);
			mass[終点.y][終点.x].set(Mass::GOAL);
			return true;
		}

		Point dirs[] = { {0, 1}, {0, -1}, {1, 0}, {-1, 0} };

		for (const auto& d : dirs)
		{
			Point next = { current.pos.x + d.x, current.pos.y + d.y };

			if (next.y < 0 || next.y >= height || next.x < 0 || next.x >= width) continue;
			if (!mass[next.y][next.x].canMove()) continue;

			float stepCost = mass[next.y][next.x].getCost();
			float tentative_g = gScore[current.pos.y][current.pos.x] + stepCost;


			if (tentative_g < gScore[next.y][next.x])
			{
				parentMap[next.y][next.x] = current.pos;
				gScore[next.y][next.x] = tentative_g;

				float h = (float)(std::abs(next.x - 終点.x) + std::abs(next.y - 終点.y));

				bool found = false;
				for (auto& node : openList) 
				{
					if (node.pos == next) 
					{
						node.g = tentative_g;
						node.f = tentative_g + h;
						found = true;
						break;
					}
				}
				if (!found) 
				{
					openList.push_back({ next, tentative_g, tentative_g + h });
				}
			}
		}
	}
	return false;
}
