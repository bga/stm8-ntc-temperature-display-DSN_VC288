/*
  Copyright 2021 Bga <bga.email@gmail.com>

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

// #define BGA__ENABLE_STATIC_PRINT

#include <!cpp/wrapper/cstring>
// #include <STM8S_StdPeriph_Driver/stm8s.h>
#include <delay.h>

#include <!mcu/Display/Text/_7Segment/_7Segment_CommonCathode.h>
#include <!mcu/Sensor/Analog/NtcThermistor.h>

#include <!hal/Adc.h>
#include <!hal/Pin/PushPull.h>
#include <!hal/Pin/PullHiZ.h>
#include <!hal/Pin/AdcWithEnable.h>

#include <!cpp/bitManipulations.h>
#include <!cpp/Binary_values_8_bit.h>
#include <!cpp/RunningAvg.h>
#include <!cpp/newKeywords.h>

#include "config.h"
#include "_7SegmentsFont.h"

#ifndef NDEBUG
  #define debug if(1)
#else
  #define debug if(0)
#endif


#ifndef UINT8_MAX
	#define UINT8_MAX 0xFF
#endif // UINT8_MAX

#ifndef UINT16_MAX
	#define UINT16_MAX 0xFFFF
#endif // UINT16_MAX

namespace App {

using namespace Bga::Mcu::Hal;


enum {
	UserError_badEeprom = 0,
};

template<UIntMax ticksCountPerSAproxArg, UInt arrMaxArg, UInt prescalerMaxArg> struct TimerCalc {
	enum {
		ticksCountPerSAprox = ticksCountPerSAproxArg,
		arrMax = arrMaxArg,
		prescalerMax = prescalerMaxArg,

		_2PowPrescalerAprox = F_CPU / ticksCountPerSAprox / arrMax,
		prescalerPossibleMax = log2Static(_2PowPrescalerAprox),
		prescaler = BGA__MATH__MIN(prescalerPossibleMax, prescalerMax),

		arr = F_CPU / (1UL << prescaler) / ticksCountPerSAprox,
		ticksCountPerSReal = F_CPU / (1UL << prescaler) / arr,
	};

	static_assert_lt(0, arr);
	static_assert_lte(arr, arrMax);
};

typedef TimerCalc<1600UL, (1UL << 16) - 1, (1UL << 4) - 1> Tim2Calc;
typedef TimerCalc<1600UL, (1UL << 8) - 1, (1UL << 3) - 1> Tim4Calc;

#define msToTicksCount(msArg) (Tim4Calc::ticksCountPerSReal * (msArg) / 1000UL)

void Timer2_init() {
	using namespace ::STM8S_StdPeriph_Lib;
	TIM2->PSCR = Tim2Calc::prescaler;
	// TIM2->EGR = TIM2_EGR_UG;

	TIM2->ARRH = U8(Tim2Calc::arr >> 8);
	TIM2->ARRL = U8(Tim2Calc::arr);

	setBitMask(TIM2->IER, TIM2_IER_UIE); // Enable Update Interrupt
	setBitMask(TIM2->CR1, TIM2_CR1_CEN); // Enable TIM2
	clearBitMask(TIM2->SR1, TIM2_SR1_UIF);
}
void Timer4_init() {
	using namespace ::STM8S_StdPeriph_Lib;
	TIM4->PSCR = Tim4Calc::prescaler;

	TIM4->ARR = Tim4Calc::arr;

	setBitMask(TIM4->IER, TIM4_IER_UIE); // Enable Update Interrupt
	setBitMask(TIM4->CR1, TIM4_CR1_CEN); // Enable TIM4
}

#if 1
template<class IntAtg>
IntAtg divmod10(IntAtg* in) {
	IntAtg div = *in  / 10;
	IntAtg mod = *in % 10;

	*in = div;
	return mod;
}
#else
FU16 divmod10(FU16& in) {
  // q = in * 0.8;
  FU16 q = (in >> 1) + (in >> 2);
  q = q + (q >> 4);
  q = q + (q >> 8);
//  q = q + (q >> 16);  // not needed for 16 bit version

  // q = q / 8;  ==> q =  in *0.1;
  q = q >> 3;

  // determine error
  FU16 r = in - ((q << 3) + (q << 1));   // r = in - q*10;
  FU16 div = q;
  FU16 mod = ((r > 9) ? ++div, r - 10 : r);

  in = div;

  return mod;
}
#endif // 1


Bool AdcUser_checkWrongValue(Adc_Value v) {
	return Config::AdcUser_minValue <= v && v <= Config::AdcUser_maxValue;
}

typedef ::Bga::Mcu::Display::Text::_7Segment_CommonCathode<6, Config::Display> Display;

Display display;

FU16 get10Power(FU16 x, FU16 max) {
	FU16 ret = 1;
	while(ret <= max && 1000 <= x) {
		x /= 10;
		ret *= 10;
	}

	return ret;
}

typedef FI16 FI10_6;
typedef U8 U2_6;

typedef U16 Um9_25;

enum TemperatureType {
	TemperatureType_celsius,
	TemperatureType_fahrenheit,
};

struct Settings {
	struct AdcUserFix {
		::Bga::Mcu::Sensor::Analog::NtcThermistor<U16> m_ntcThermistor;
		Bool isValid() const {
			return m_ntcThermistor.m_rIce_ohm != 0xFFFF && m_ntcThermistor.m_rIce_ohm != 0;
		}
	};

	AdcUserFix tempAdcFix;
	U16 rDiv;
	U16 displayUpdatePeriod;
	U2_6 tempHysteresis;
	struct HLDisplayData {
		U16 nameDisplayEndTime;
		U16 valueDisplayEndTime;
	} hlDisplayData;
	/* TemperatureType */ U8 temperatureType;
};

STM8S_STDPERIPH_LIB__EEPROM const Settings defaultSettings = {
	.tempAdcFix = {
		.m_ntcThermistor = {
			.m_rIce_ohm = 35000,
			.m_oneDivB = Um9_25(5753),
		}
	},
	.rDiv = 10000,
	.displayUpdatePeriod = msToTicksCount(500UL),
	.tempHysteresis = U2_6(0.045 * (1 << 6)),
	.temperatureType = TemperatureType_celsius,
	.hlDisplayData = {
		.nameDisplayEndTime = msToTicksCount(500UL),
		.valueDisplayEndTime = msToTicksCount(1500UL),
	}
};
Settings const& settings = ((Settings*)(&defaultSettings))[0];

FU16 rrToRh(FU16 adc, FU16 rL) {
	return FU32(rL) * (Adc_maxValue - adc) / adc;
//	return rL * (Adc_maxValue / adc - 1);
}


void displayDecrimal(FU16 x, FU8* dest) {
	forDec(int, i,  0, 3) {
		dest[i] = _7SegmentsFont::digits[divmod10(&x)];
	}
}

void displayDecrimal6(FU16 x, FU8* dest) {
	forDec(int, i,  0, 6) {
		dest[i] = _7SegmentsFont::digits[divmod10(&x)];
	}
}

void displayHwError(FU8* dest) {
	dest[0] = _7SegmentsFont::H;
	dest[1] = _7SegmentsFont::H;
	dest[2] = _7SegmentsFont::H;
}
void displaySoftError(FU16 errorCode, FU8* dest) {
	dest[0] = _7SegmentsFont::E;
	forDec(int, i,  1, 3) {
		dest[i] = _7SegmentsFont::digits[divmod10(&errorCode)];
	}
}

void display_fixLastDigit(FU16 x, FU8* dest, void (*display)(FU16 x, FU8* dest)) {
	//# small value
	if(1000 <= x) {
		display(x, dest);
	}
	else {
		FU8 newDigits[3];
		display(x, newDigits);
		//# prevent display last digit small changes (ADC error)
		if(newDigits[0] == dest[0] && newDigits[1] == dest[1]) {
			//# keep old digits
		}
		else {
			memcpy(dest, newDigits, 3);
		}
	}
}

namespace TemperatureConvertor {
	FI16 celsiusToFahrenheit(FI16 t) {
		return ((FI32(t) * 58982) >> 15) + 32 * 10;
	}
}

FI16 User_convertTemperature(FI16 t) {
	switch(settings.temperatureType) {
		case(TemperatureType_celsius): break;
		case(TemperatureType_fahrenheit): t = TemperatureConvertor::celsiusToFahrenheit(t); break;
		default: break;
	}
	return t;
}

//# 0 < x <= 999(999mA) => xx.xC
//# 999(99.9C) < x <= 9999(999.9C) => xxx / 10 C
void displayTemp(FI16 x, FU8* dest) {
	const FI16 xVal = x;
	const FU16 xValAbs = Math_abs(x);

	x = xValAbs;
	(1000 <= xVal) && (divmod10(&x));

	forDec(int, i,  0, 3) {
		dest[i] = _7SegmentsFont::digits[divmod10(&x)];
	}

	if(xValAbs < 1000) {
		dest[1] |= _7SegmentsFont::dot;
	};

	if(xVal < 0) {
		dest[2] |= _7SegmentsFont::dot;
	};


}

enum {
	HLDisplayData_h1 = 0,
	HLDisplayData_l1,
	HLDisplayData_h7,
	HLDisplayData_l7,
	HLDisplayData_h30,
	HLDisplayData_l30,
};

const FI10_6 HLDisplayData_notFilledMagic = FI10_6(0x8000);


struct HLDisplayData {
	const char name[3];
	FI16 temp;
} hlDisplayData[6] = {
	[HLDisplayData_h1] = { { _7SegmentsFont::H, _7SegmentsFont::d1, 0  }, HLDisplayData_notFilledMagic },
	[HLDisplayData_l1] = { { _7SegmentsFont::L, _7SegmentsFont::d1, 0  }, 222 },
	[HLDisplayData_h7] = { { _7SegmentsFont::H, _7SegmentsFont::d7, 0  }, 333 },
	[HLDisplayData_l7] = { { _7SegmentsFont::L, _7SegmentsFont::d7, 0  }, 444 },
	[HLDisplayData_h30] = { { _7SegmentsFont::H, _7SegmentsFont::d3, _7SegmentsFont::d0 }, 444 },
	[HLDisplayData_l30] = { { _7SegmentsFont::L, _7SegmentsFont::d3, _7SegmentsFont::d0 }, 667 },
};

inline Bool HLDisplayData_isNotFilled() {
	return hlDisplayData[HLDisplayData_h1].temp == HLDisplayData_notFilledMagic;
}

FU16 hlDisplayData_ticksCount = -1;
FU16 hlDisplayData_index = 0;


FI10_6 lastTemp = -1;


FU16 ticksCountLive = 0;
FU16 displayTicksCount = 0;


enum { adcFetchPeriod = (1 << (Config::AdcUser_fetchSpeedPrescaler)) };
FU16 nextAdcTickCount = 0;

RunningAvg<FU16[Config::AdcUser_adcsSize], FU32> tempAdcRunningAvg;
Bool tempAdc_isHwError;

FU8 display_index = 0;

void sysClockThread() {
	ticksCountLive += 1;
}

void displayThread() {
	FU16 ticksCount = ticksCountLive;

	#if 1
	display.updateManual(display_index);
	cycleInc(display_index, 6);
	#endif // 1
}

void measureThread() {
	FU16 ticksCount = ticksCountLive;

	#if 0
		displayDecrimal(ticksCount, &(display.displayChars[3]));
	#endif // 1

	#if 1
	if(ticksCount == nextAdcTickCount) {
		nextAdcTickCount += adcFetchPeriod;

		Config::tempAdcGpioVccPort.on();
		Adc_Value t = Config::tempAdcGpioPort.readSync();
		Config::tempAdcGpioVccPort.off();

		if(AdcUser_checkWrongValue(t)) {
			tempAdcRunningAvg.add(t);
			tempAdc_isHwError = false;
		}
		else {
			tempAdc_isHwError = true;
		}
	};
	#endif

	#if 1
	if(settings.displayUpdatePeriod <= (displayTicksCount += 1) ) {
		if(tempAdc_isHwError) {
			displayHwError(&(display.displayChars[Config::tempDisplayCharIndex]));
		}
		else if(!settings.tempAdcFix.isValid()) {
			displaySoftError(UserError_badEeprom, &(display.displayChars[Config::tempDisplayCharIndex]));
		}
		else {
			FU16 adcAvg = tempAdcRunningAvg.computeAvg();
			debug displayDecrimal(adcAvg, &(display.displayChars[3]));

			FU16 rH = rrToRh(adcAvg, settings.rDiv);
			FI10_6 temp = settings.tempAdcFix.m_ntcThermistor.convert(rH);

			if(settings.tempHysteresis < Math_abs(temp - lastTemp)) {
				lastTemp = temp;
				FI16 userTemp = ((FI32(temp) * 10) >> 6);

				userTemp = User_convertTemperature(userTemp);

				displayTemp(userTemp, &(display.displayChars[Config::tempDisplayCharIndex]));
			};
		}
	}
	#endif

	#if 1
	if(HLDisplayData_isNotFilled()) {
		//# display nothing
	}
	else {
		if(0 == hlDisplayData_ticksCount) {
			memcpy(&(display.displayChars[Config::hlDisplayDataCharIndex]), hlDisplayData[hlDisplayData_index].name, sizeof(hlDisplayData[hlDisplayData_index].name));
		}
		else if(settings.hlDisplayData.nameDisplayEndTime == hlDisplayData_ticksCount) {
			FI16 userTemp = hlDisplayData[hlDisplayData_index].temp;

			userTemp = User_convertTemperature(userTemp);

			displayTemp(userTemp, &(display.displayChars[Config::hlDisplayDataCharIndex]));
		}
		else if(settings.hlDisplayData.valueDisplayEndTime == hlDisplayData_ticksCount) {
			cycleInc(hlDisplayData_index, arraySize(hlDisplayData));
			hlDisplayData_ticksCount = -1;
		}
		hlDisplayData_ticksCount += 1;
	}
	#endif


	if(0) debug {
		if((ticksCount & bitsCountToMask(7)) == 0) {
			display.displayChars[0] ^= _7SegmentsFont::dot;
		};
	}
}

BGA__MCU__HAL__ISR(STM8S_STDPERIPH_LIB__TIM4_ISR) {
	clearBitMask(::STM8S_StdPeriph_Lib::TIM4->SR1, ::STM8S_StdPeriph_Lib::TIM4_SR1_UIF);

	displayThread();
	measureThread();
	sysClockThread();
}
BGA__MCU__HAL__ISR(STM8S_STDPERIPH_LIB__TIM2_OVF_ISR) {
	clearBitMask(::STM8S_StdPeriph_Lib::TIM2->SR1, ::STM8S_StdPeriph_Lib::TIM2_SR1_UIF);

	measureThread();
}

void Clock_setCpuFullSpeed() {
	using namespace ::STM8S_StdPeriph_Lib;
	CLK->CKDIVR = 0;
}

void Hw_enable() {
	using namespace ::STM8S_StdPeriph_Lib;
	//# CLK->PCKENR* by default has all bits set. We need only enable clocking for used peripheral. So direct assignment
	CLK->PCKENR1 = CLK_PCKENR1_TIM2 | CLK_PCKENR1_TIM4;
	CLK->PCKENR2 = CLK_PCKENR2_ADC;
}


void main() {

	Clock_setCpuFullSpeed();
	Hw_enable();
	display.init();

	//# test data for display
	forInc(FU8, i, 0, 6) {
		display.displayChars[i] = (i < 3) ? _7SegmentsFont::digits[i] : 0;
	}


//	displaySoftError(12, &(display.displayChars[0]));
//	displayTemp(FI10_6(4.2 * (1UL << 6)), &(display.displayChars[0]));
//	displayTemp(FI10_6(24.2 * (1UL << 6)), &(display.displayChars[0]));
//	displayTemp(FI10_6(124.2 * (1UL << 6)), &(display.displayChars[0]));
//?	displayDecrimal6(settings.displayUpdatePeriod, &(display.displayChars[0]));

	Adc_init();

	Config::tempAdcGpioPort.init();
	Config::tempAdcGpioVccPort.init();

	// Timer2_init();
	Timer4_init();
	::STM8S_StdPeriph_Lib::enableInterrupts();

	while(1) {
//		delay_ms(500);
//		display.displayChars[0] ^= _7SegmentsFont::dot;
		::STM8S_StdPeriph_Lib::wfi();
	}
//	while(1) __wait_for_interrupt();
}

} //# namespace

void main() {
	App::main();
}
