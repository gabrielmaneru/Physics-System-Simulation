/**
 * @file engine.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Main loop
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include "engine.h"
#include "window.h"
#include "physics.h"
#include "drawer.h"
#include "editor.h"

/**
 * Initialize the engine
**/
bool engine::initialize()
{
	if (!window.initialize()) return false;
	if (!physics.initialize()) return false;
	if (!drawer.initialize()) return false;
	if (!editor.initialize()) return false;
	return true;
}

/**
 * Loop until exit
**/
void engine::update()
{
	do
	{
		window.update();
		physics.update();
		drawer.render();
		editor.drawGUI();
		window.present();
	}
	while (!window.should_exit());
}

/**
 * Shutdown the engine
**/
void engine::shutdown()
{
	editor.shutdown();
	drawer.shutdown();
	physics.shutdown();
	window.shutdown();
}
