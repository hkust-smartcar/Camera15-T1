/*
 * launcher.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

namespace camera
{

class Launcher
{
public:
	void Run();

private:
	void StartApp(const int id);
};

}
