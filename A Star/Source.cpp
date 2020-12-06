#include <iostream>
#include <string>
#include <algorithm>

using namespace std;

#include "olcConsoleGameEngine.h"

class AStar : public olcConsoleGameEngine
{
public:
	AStar()
	{
		m_sAppName = L"A* Path Finding";
	}

private:
	struct sNode
	{
		bool bObstacle = false;
		bool bVisited = false;
		float fGlobalGoal;
		float fLocalGoal;
		int x, y;
		vector<sNode*> vecNeighbours;
		sNode* parent;
	};

	sNode *nodes = nullptr;
	int nMapWidth = 16;
	int nMapHeight = 16;

	sNode *nodeStart = nullptr;
	sNode *nodeEnd = nullptr;

protected:
	virtual bool OnUserCreate()
	{
		nodes = new sNode[nMapWidth * nMapHeight];
		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapHeight; y++) {
				nodes[y * nMapWidth + x].x = x;
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].bObstacle = false;
				nodes[y * nMapWidth + x].bVisited = false;
				nodes[y * nMapWidth + x].parent = nullptr;
			}
		}
		
		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapHeight; y++) {
				if (y > 0) 
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + x]);
				if (y < nMapHeight - 1)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + x]);
				if (x > 0)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[y * nMapWidth + (x - 1)]);
				if (x < nMapWidth -1)
					nodes[y * nMapWidth + x].vecNeighbours.push_back(&nodes[y * nMapWidth + (x + 1)]);
			}
		}
		
		nodeStart = &nodes[(nMapHeight / 2) * nMapWidth + 1];
		nodeEnd = &nodes[(nMapHeight / 2) * nMapWidth + nMapWidth - 2];
		return true;
	}

	void Solve_AStar() {
		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapWidth; y++) {
				nodes[y * nMapWidth + x].bVisited = false;
				nodes[y * nMapWidth + x].fLocalGoal = INFINITY;
				nodes[y * nMapWidth + x].fGlobalGoal = INFINITY;
				nodes[y * nMapWidth + x].parent = nullptr;
			}
		}

		auto distance = [](sNode* a, sNode* b) {
			return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
		};

		auto heuristic = [distance](sNode* a, sNode* b) {
			return distance(a, b);
		};

		sNode* nodeCurrent = nodeStart;
		nodeStart->fLocalGoal = 0.0f;
		nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

		list<sNode*> listNotTestedNodes;
		listNotTestedNodes.push_back(nodeStart);

		while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd) {
			listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs) { return lhs->fGlobalGoal < rhs->fGlobalGoal; });

			while (!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
				listNotTestedNodes.pop_front();

			if (listNotTestedNodes.empty())
				break;

			nodeCurrent = listNotTestedNodes.front();
			nodeCurrent->bVisited = true;

			for (auto nodeNeighbour : nodeCurrent->vecNeighbours) {
				if (!nodeNeighbour->bVisited && !nodeNeighbour->bObstacle) {
					listNotTestedNodes.push_back(nodeNeighbour);
				}

				float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

				if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal) {
					nodeNeighbour->parent = nodeCurrent;
					nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
					nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
				}
			}
		}
	}

	virtual bool OnUserUpdate(float fElapsedTime)
	{
		int nNodeSize = 9;
		int nNodeBorder = 2;

		int nSelectedNodeX = m_mousePosX / nNodeSize;
		int nSelectedNodeY = m_mousePosY / nNodeSize;

		if (m_mouse[0].bReleased) {
			if (nSelectedNodeX >= 0 && nSelectedNodeX < nMapWidth) {
				if (nSelectedNodeY >= 0 && nSelectedNodeY < nMapHeight) {
					if (m_keys[VK_SHIFT].bHeld)
						nodeStart = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
					else if (m_keys[VK_CONTROL].bHeld)
						nodeEnd = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
					else
						nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle = !nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle;

					Solve_AStar();
				}
			}
		}

		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');

		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapHeight; y++) {
				for (auto n : nodes[y * nMapWidth + x].vecNeighbours) {
					DrawLine(x * nNodeSize + nNodeSize / 2, y * nNodeSize + nNodeSize / 2,
						n->x * nNodeSize + nNodeSize / 2, n->y * nNodeSize + nNodeSize / 2, PIXEL_SOLID, BG_CYAN);
				}
			}
		}

		for (int x = 0; x < nMapWidth; x++) {
			for (int y = 0; y < nMapHeight; y++) {
				Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder,
					(x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder,
					PIXEL_HALF, nodes[y * nMapWidth + x].bObstacle ? BG_MAGENTA : BG_DARK_CYAN);

				if (nodes[y * nMapWidth + x].bVisited)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder, (x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder, PIXEL_HALF, BG_GREY);

				if (&nodes[y * nMapWidth + x] == nodeStart)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder, (x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder, PIXEL_HALF, BG_GREEN);

				if (&nodes[y * nMapWidth + x] == nodeEnd)
					Fill(x * nNodeSize + nNodeBorder, y * nNodeSize + nNodeBorder, (x + 1) * nNodeSize - nNodeBorder, (y + 1) * nNodeSize - nNodeBorder, PIXEL_SOLID, FG_RED);
			}
		}

		if (nodeEnd != nullptr) {
			sNode* p = nodeEnd;
			while (p->parent != nullptr) {
				DrawLine(p->x * nNodeSize + nNodeSize / 2, p->y * nNodeSize + nNodeSize / 2,
					p->parent->x * nNodeSize + nNodeSize / 2, p->parent->y * nNodeSize + nNodeSize / 2, PIXEL_SOLID, FG_YELLOW);
				p = p->parent;
			}
		}

		return true;
	}
};

int main()
{
	AStar game;
	game.ConstructConsole(160, 160, 6, 6);
	game.Start();
	return 0;
}