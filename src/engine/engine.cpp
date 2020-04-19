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
	if (!window.initialize()) // Create window
		return false;
	if (!drawer.initialize()) // Initialize OpenGL
		return false;
	if (!editor.initialize()) // Create scene
		return false;
	return true;
}

/**
 * Loop until exit
**/
void engine::update()
{
	do
	{
		// Update System
		window.update();	// Update window
		editor.update();	// Update editor
		physics.update();	// Integrate physics

		// Render Screen
		drawer.render();	// Render primitives
		editor.drawGui();	// Render ImGui
		window.present();	// Swapbuffers
	}
	while (!window.should_exit());
}

/**
 * Shutdown the engine
**/
void engine::shutdown()
{
	editor.shutdown();
	window.shutdown();
}
