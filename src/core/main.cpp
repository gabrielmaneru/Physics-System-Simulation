/**
 * @file main.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Main program file
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include <engine/engine.h>

int main(int, char**)
{
	// Initialize Engine
	if (!engine::initialize()) 
		return -1;
	// Update Engine Loop
	engine::update();
	// Shutdown Engine
	engine::shutdown();
	return 0;
}
