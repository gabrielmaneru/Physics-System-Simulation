#include "engine.h"
#include "window.h"
#include "drawer.h"

bool engine::initialize()
{
	if (!window.initialize()) return false;
	if (!drawer.initialize()) return false;
	return true;
}

void engine::update()
{
	do
	{
		window.update();
		// physics update
		drawer.render();
		window.present();
	}
	while (!window.should_exit());
}

void engine::shutdown()
{
	drawer.shutdown();
	window.shutdown();
}
