/**
 * @file main_gtest.cpp
 * @author Gabriel Maneru, gabriel.m, gabriel.m@digipen.edu
 * @date 01/28/2020
 * @brief Main program file for tests
 * @copyright Copyright (C) 2020 DigiPen Institute of Technology.
**/
#include <gtest/gtest.h>

GTEST_API_ int main(int argc, char** argv)
{
	// Initialize GTest
	testing::InitGoogleTest(&argc, argv);
	// Run all test
	auto ret = RUN_ALL_TESTS();
	return ret;
}
