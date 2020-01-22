#include "engine.h"
#include "drawer.h"

bool engine::initialize()
{
	if (!drawer.initialize()) return false;
	return true;
}

void engine::update()
{
	do
	{
		drawer.update_window();
		// physics update
		drawer.render_window();
	}
	while (!drawer.should_exit());
}

void engine::shutdown()
{
	drawer.shutdown();
}
