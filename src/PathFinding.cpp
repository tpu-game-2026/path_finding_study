#include "board.h"

int main()
{
	std::vector<std::string> map_data = {
	"############################",
	"#                          #",
	"# $$$$$$$$$$$$$$$$$$$$$$   #",
	"# ~~~~~~~~~~~~~~~~~~~~~~   #",
	"# ~~~~~~~~~~~~~~~~~~~~~~   #",
	"# ~~~~~~~~~~~~~~~~~~~~~~   #",
	"#                          #",
	"#                          #",
	"#                          #",
	"#                          #",
	"############################",
	};
	Board board(map_data);

	// 経路探索
	Point 始点 = { 24, 5 };
	Point 終点 = { 2, 5 };
	std::vector<std::vector<Mass>> mass = board.setup();
	board.find(始点, 終点, mass);

	board.show(mass);// 経路の可視化

	system("PAUSE");// 終る前に入力をうながす
}
