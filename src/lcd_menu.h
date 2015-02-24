/*
 * lcd_menu.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "libsc/k60/lcd_typewriter.h"

#include <utility>
#include <vector>

namespace libsc
{
namespace k60
{

class St7735r;

}
}

namespace camera
{

class LcdMenu
{
public:
	typedef libsc::k60::St7735r Lcd;

	explicit LcdMenu(Lcd *lcd);

	void AddItem(const Uint id, const char *label_literal);
	void Select(const int position);
	Uint GetSelectedPosition() const
	{
		return m_select + m_first;
	}
	Uint GetSelectedId() const;

private:
	void Redraw();

	Lcd *m_lcd;
	libsc::k60::LcdTypewriter m_writer;

	Uint m_first;
	Uint m_select;
	const Uint m_max_line;

	std::vector<std::pair<Uint, const char*>> m_items;
};

}
