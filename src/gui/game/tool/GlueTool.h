#pragma once
#include "Tool.h"

class GameModel;

class GlueTool: public Tool
{
	GameModel &gameModel;

	friend class SignWindow;

public:
	GlueTool(GameModel &model):
		Tool(0, "GLUE", "Make a physics object by glueing together pixels.",
			0x000000_rgb, "PHYS_GLUE", GlueTool::GetIcon
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
