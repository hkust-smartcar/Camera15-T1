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

struct SystemRes;

}

namespace camera
{

class App
{
public:
	explicit App(SystemRes *res)
			: m_res(res)
	{}

	virtual ~App()
	{}

	virtual void Run() = 0;

protected:
	SystemRes* GetSystemRes()
	{
		return m_res;
	}

private:
	SystemRes *m_res;
};

}
