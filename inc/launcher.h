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

	{
			data[0] = 1275.0f;
			data[1] = 0.43f;
			data[2] = 0.0f;
			data[3] = 0.045f;
	}

	void Run();

private:
	void StartApp(const int id);
	void setParam(const int id);
	float data[4];
};

}

/*
 	 	 	around 2m/s:
  			data[0] = 1200.0f;
			data[1] = 0.42f;
			data[2] = 0.0f;
			data[3] = 0.045f;

			faster?
  			data[0] = 1350.0f;
			data[1] = 0.46f;
			data[2] = 0.0f;
			data[3] = 0.046f;
*/
