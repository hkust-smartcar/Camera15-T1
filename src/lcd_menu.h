/*
 * lcd_menu.h
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include "libsc/lcd_typewriter.h"

#include <utility>
#include <vector>

namespace libsc
{

class St7735r;

}

namespace camera
{

class LcdMenu
{
public:
	typedef libsc::St7735r Lcd;

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
	libsc::LcdTypewriter m_writer;

	Uint m_first;
	Uint m_select;
	const Uint m_max_line;

	Uint m_offset_y;

	std::vector<std::pair<Uint, const char*>> m_items;
};

}
