#include "BoxTool.h"
#include "simulation/Simulation.h"
#include "gui/Style.h"
#include "gui/interface/Window.h"
#include "gui/interface/Button.h"
#include "gui/interface/Label.h"
#include "gui/interface/Textbox.h"
#include "gui/interface/DropDown.h"
#include "gui/game/GameModel.h"
#include "graphics/Graphics.h"

std::unique_ptr<VideoBuffer> BoxTool::GetIcon(int toolID, Vec2<int> size)
{
	auto texture = std::make_unique<VideoBuffer>(size);
	texture->DrawRect(size.OriginRect(), 0xA0A0A0_rgb);
	texture->DrawRect(size.OriginRect(), 0xA0A0A0_rgb);
	texture->BlendChar((size / 2) - Vec2(5, 5), 0xE021, 0x204080_rgb .WithAlpha(0xFF));
	// texture->BlendChar((size / 2) - Vec2(5, 5), 0xE020, 0xFFFFFF_rgb .WithAlpha(0xFF));
	return texture;
}

void BoxTool::Click(Simulation * sim, Brush const &brush, ui::Point position)
{
	sim->physicsSimulation.CreateBody(position);
}
