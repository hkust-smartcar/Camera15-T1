/*
 * lcd_menu.cpp
 *
 * Author: Ben Lai, Ming Tsang, Peggy Lau
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <utility>
#include <vector>

#include <libsc/k60/lcd_typewriter.h>
#include <libsc/k60/st7735r.h>
#include <libutil/misc.h>

#include "lcd_menu.h"

using namespace libsc::k60;
using namespace std;

namespace camera
{

namespace
{

LcdTypewriter::Config GetTypeWriterConfig(LcdMenu::Lcd *lcd)
{
	LcdTypewriter::Config product;
	product.lcd = lcd;
	return product;
}

}

LcdMenu::LcdMenu(Lcd *lcd)
		: m_lcd(lcd),
		  m_writer(GetTypeWriterConfig(lcd)),
		  m_first(0),
		  m_select(0),
		  m_max_line(Lcd::GetH() / LcdTypewriter::GetFontH())
{}

void LcdMenu::AddItem(const Uint id, const char *label_literal)
{
	m_items.push_back(make_pair(id, label_literal));
}

void LcdMenu::Select(const int position)
{
	m_select = libutil::Clamp<Uint>(0, position, m_items.size() - 1);
	m_first = std::max<int>(m_select - (int)m_max_line + 1, m_first);
	Redraw();
}

Uint LcdMenu::GetSelectedId() const
{
	return m_items[m_select].first;
}

void LcdMenu::Redraw()
{
	Uint y = 0;
	for (Uint i = m_first; i < m_items.size() && i < m_max_line; ++i)
	{
		//m_lcd->SetRegion(Lcd::Rect(0, y, Lcd::GetW(), LcdTypewriter::GetFontH()));
		m_writer.SetBgColor((i == m_select) ? 0x35BC : 0);
		m_writer.WriteString(m_items[i].second);
		y += LcdTypewriter::GetFontH();
	}
}

}
