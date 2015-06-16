/*
 * Imp.h
 *
 *  Created on: 2015年6月12日
 *      Author: Benlai
 */

#ifndef CAMERA15VER2_SRC_IMP_H_
#define CAMERA15VER2_SRC_IMP_H_


#include "libbase/misc_types.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libsc/St7735r.h"
#include <libsc/system.h>
#include <stdlib.h>

using namespace libsc::k60;
using namespace libsc;



class Imp {
public:
	Imp();

	virtual ~Imp();

	void medianFilterPrint(const Byte* src, St7735r *lcd);

	void medianFilter (const Byte* src);


private:


//	void medianFilter (const Byte* src);

	int GetPixel(const Byte* src, const uint8_t x, const uint8_t y);

	int BoolGetPixel(const bool* src, const uint8_t x,const uint8_t y);

	bool mf[4524]; //58*78




};

#endif /* CAMERA15VER2_SRC_IMP_H_ */
