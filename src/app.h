/*
 * app.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

namespace camera
{

class App
{
public:
	virtual ~App()
	{}

	virtual void Run() = 0;
};

}
