/*
 * camera_test_app.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "app.h"

namespace camera
{

class CameraTestApp : public App
{
public:
	explicit CameraTestApp(SystemRes *res)
			: App(res)
	{}

	void Run() override;
};

}
