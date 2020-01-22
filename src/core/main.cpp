#include <engine/engine.h>

int main(int, char**)
{
	if (!engine::initialize())
		return 1;
	engine::update();
	engine::shutdown();
	return 0;
}
