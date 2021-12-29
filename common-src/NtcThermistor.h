/*
	Copyright 2020 Bga <bga.email@gmail.com>

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#pragma once

#include <!cpp/common.h>
#include <!cpp/Math/Fp/log2fix.h>
#include <!cpp/TestRunner.h>

namespace Bga { namespace Mcu { namespace Sensor { namespace Analog { 

#pragma push_macro("Self")

#undef Self
#define Self NtcThermistor

template<class RArg> 
struct Self;

template<> 
struct Self<U16> {
	typedef FI16 FI10_6;
	typedef U16 Um9_25;

	U16 m_rIce_ohm; //# measure ntc resistance in ice
	Um9_25 m_oneDivB; //# power coeff
	#if 0
		/* maxima code */
		/* r in ice */
		t0: 0;
		r0: 35000;

		/* r of your body */
		t1: 36.6;
		r1: 6090;

		T0: 273.15;
		temp(t, r, a, b) := 1 / (t + T0) - (a + b * log(r) / log(2));
		data: solve([temp(t0, r0, a, b), temp(t1, r1, a, b)], [a, b]), bfloat;
		
		/* result m_oneDivB */
		data[1][2] * 2 ** 25;
	#endif // 0
	
	FI10_6 convert(U16 r_ohm) const {
		//# accepted range - 3.25
		FU32 ratio = ((FU32(r_ohm) << 16) /* 16.16 */ / m_rIce_ohm /* 16.0 */) /* 16.16 */;
		FI16 logR = Math::Fp::log2fix(U16(ratio >> 4 /* 18.14 */) /* 2.14 */, 16 - 4) /* 4.12 */;
		FI16 oneDivAbsTemp = I16(((FU32(m_oneDivB /* -9.25 */) * logR /* 4.12 */) /* -5.37 */ >> 14) /* -7.23 */ + FI16(I16(1.0 / 273.15 * (uintmax_t(1) << 23))));
		
		return FI16(FI32(1UL << (30 - 1)) /* 3.29 */ / oneDivAbsTemp /* -7.23 */) /* 10.6 */ - FI16(I16(273.15 * (uintmax_t(1) << 6))) /* 10.6 */;
	}
};

} }  } } //# namespace


#ifdef BGA__TESTRUNNER_ON
example(BGA__STR(Self)) {
	using namespace Bga::Mcu::Sensor::Analog;
	
	typedef Self<U16> X;
	X x; {
		x.m_r0 = 10000;
		x.m_oneDivB = X::Um9_25(6000);
	}
	
	//# t at r0 should be zero
	assert_eq(x.convert(10000), 0);
	
	//# double step up and down. Some precalculated data
	assert_eq(x.convert(20000), -814);
	assert_eq(x.convert(5000), 898);
}
#endif

#pragma pop_macro("Self")
