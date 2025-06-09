#pragma once
#include "Tool.h"

class GameModel;

class BoxTool: public Tool
{
	GameModel &gameModel;

	friend class SignWindow;

public:
	BoxTool(GameModel &model):
		Tool(0, "BOX", "Make a box with 2d physics",
			0x000000_rgb, "PHYS_BOX", BoxTool::GetIcon
		),
		gameModel(model)
	{}

	static std::unique_ptr<VideoBuffer> GetIcon(int toolID, Vec2<int> size);
	void Click(Simulation * sim, Brush const &brush, ui::Point position) override;
	void Draw(Simulation * sim, Brush const &brush, ui::Point position) override { }
	void DrawLine(Simulation * sim, Brush const &brush, ui::Point position1, ui::Point position2, bool dragging) override { }
	void DrawRect(Simulation * sim, Brush const &brush, ui::Point position1, ui::Point position2) override { }
	void DrawFill(Simulation * sim, Brush const &brush, ui::Point position) override { }
};
