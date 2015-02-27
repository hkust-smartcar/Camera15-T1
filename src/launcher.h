/*
 * launcher.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "app.h"

namespace camera
{

class Launcher : public App
{
public:
	explicit Launcher(SystemRes *res)
			: App(res)
	{}

	void Run();

private:
	void StartApp(const int id);
};

}
