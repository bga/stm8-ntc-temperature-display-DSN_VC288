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

//# explicit include for codeblocks
#include "../pre-config.h"

#pragma diag_suppress=Pe047
//# fix for codeblocks code competent
#define __cplusplus __cplusplus
#pragma diag_default=Pe047

#include <!cpp/wrapper/cstring>
#include <!cpp/wrapper/optional>
// #include <STM8S_StdPeriph_Driver/stm8s.h>
#include <delay.h>

#include <!mcu/Display/Text/_7Segment/_7Segment_CommonCathode.h>
#include "../common-src/NtcThermistor.h"

#include <!hal/Adc.h>
#include <!hal/Eeprom.h>
#include <!hal/Pin/PushPull.h>
#include <!hal/Pin/PullHiZ.h>
#include <!hal/Pin/PullUp.h>
#include <!hal/Pin/AdcWithEnable.h>

#include <!cpp/bitManipulations.h>
#include <!cpp/Binary_values_8_bit.h>
#include <!cpp/RunningAvg.h>
#include <!cpp/CircularBuffer.h>
#include <!cpp/Math/MinMaxRollingBinaryTreeFinder.h>
#include <!cpp/Scheduler/Stack.h>
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

Bool isDebugMode = false;

enum {
	UserError_badEeprom = 0,
};

template<UIntMax ticksCountPerSAproxArg> struct Timer4 {
	enum {
		ticksCountPerSAprox = ticksCountPerSAproxArg,
		arrMax = (1UL << 8) - 1,
		prescalerMax = (1UL << 3) - 1,

		_2PowPrescalerAprox = F_CPU / ticksCountPerSAprox / arrMax,
		prescalerPossibleMax = log2Static(_2PowPrescalerAprox),
		prescaler = BGA__MATH__MIN(prescalerPossibleMax, prescalerMax),

		arr = F_CPU / (1UL << prescaler) / ticksCountPerSAprox,
		ticksCountPerSReal = F_CPU / (1UL << prescaler) / arr,
	};

	static_assert_lt(0, arr);
	static_assert_lte(arr, arrMax);

	void init() {
		using namespace ::STM8S_StdPeriph_Lib;
		TIM4->PSCR = prescaler;

		TIM4->ARR = arr;

		setBitMask(TIM4->IER, TIM4_IER_UIE); // Enable Update Interrupt
		setBitMask(TIM4->CR1, TIM4_CR1_CEN); // Enable TIM4
	}
	Bool hasPendingInterrupt() const {
		return hasBitMask(::STM8S_StdPeriph_Lib::TIM4->SR1, ::STM8S_StdPeriph_Lib::TIM4_SR1_UIF);
	}
	Bool clearPendingInterrupt() {
		return clearBitMask(::STM8S_StdPeriph_Lib::TIM4->SR1, ::STM8S_StdPeriph_Lib::TIM4_SR1_UIF);
	}
	FU8 readCounter() const {
		return ::STM8S_StdPeriph_Lib::TIM4->CNTR;
	}
};

struct  Timer: public Timer4<Config::renderTempTask_freqAprox> {

} timer;

#define msToTicksCount(msArg) (Timer::ticksCountPerSReal * (msArg) / 1000UL)

void timerThread(Timer& timer);
BGA__MCU__HAL__ISR(STM8S_STDPERIPH_LIB__TIM4_ISR) {
	timerThread(timer);
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

static const Adc_Value Adc_maxValueOversampled = Adc_maxValue << Config::AdcUser_oversampleExtraBitsCount;


Bool AdcUser_checkWrongValue(Adc_Value v) {
	return Config::AdcUser_minValue <= v && v <= Config::AdcUser_maxValue;
}

typedef ::Bga::Mcu::Display::Text::_7Segment_CommonCathode<6, Config::Display> Display;

Display display;

#define APP__DEBUG__WRITE(C0Arg, C1Arg, C2Arg) ( \
	(display.displayChars[3] = _7SegmentsFont::C0Arg), \
	(display.displayChars[4] = _7SegmentsFont::C1Arg), \
	(display.displayChars[5] = _7SegmentsFont::C2Arg), \
0)

FU16 get10Power(FU16 x, FU16 max) {
	FU16 ret = 1;
	while(ret <= max && 1000 <= x) {
		x /= 10;
		ret *= 10;
	}

	return ret;
}

typedef FI16 FTempK10_6;
typedef I16 TempK10_6;
typedef U8 U2_6;

typedef U16 Um9_25;

enum TemperatureType {
	TemperatureType_celsius,
	TemperatureType_fahrenheit,
};

template<class ConfigArg>
struct Crc8_Bit {
	typedef ConfigArg Config;
	Config config;
	
	typedef U8 CrcValue;
	
	FU8 crc;
	
	void init() {
		this->crc = FU8(-1);
	}
	//# [https://github.com/pebble/ArduinoPebbleSerial/blob/master/utility/crc.c]
	//# MIT license
	private: FU8 updateCrc(const FU8 data, FU8 crc) const {
		crc ^= data;
		forInc(FU8, bit, 0, 8) {
			if((crc & 0x80) != 0) {
				crc <<= 1;
				crc ^= FU8(this->config.generator);
			}
			else {
				crc <<= 1;
			}
		}
		return crc;
	}
	public: void add(const FU8 data) {
		this->crc = this->updateCrc(data, this->crc);
	}
	
	FU8 getResult() const {
//		return this->updateCrc(FU8(-1), this->crc);
//		return ~this->crc;
		return this->crc;
	}
};



template<class ConfigArg>
struct Crc8_Nibble_OneTable {
	typedef ConfigArg Config;
	Config config;
	
	typedef U8 CrcValue;
	
	FU8 crc;
	
	void init() {
		this->crc = FU8(-1);
	}
	//# [https://github.com/pebble/ArduinoPebbleSerial/blob/master/utility/crc.c]
	//# MIT license
	private: FU8 updateCrc(const FU8 data, FU8 crc) const {
		for(FU8 i = 2; i > 0; i--) {
			FU8 nibble = data;
			if(i % 2 == 0) {
				nibble >>= 4;
			};
			FU8 index = nibble ^ (crc >> 4);
			crc = FU8(this->config.lookupTable[index & 0xf]) ^ (crc << 4);
		}
		
		return crc;
	}
	public: void add(const FU8 data) {
		this->crc = this->updateCrc(data, this->crc);
	}
	
	FU8 getResult() const {
//		return this->updateCrc(FU8(-1), this->crc);
//		return ~this->crc;
		return this->crc;
	}
};

struct Crc8_Config {
	static const U8 generator = B0010_1111;
	static const U8 lookupTable[16];
};

//# Optimal polynomial chosen based on
//# http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
//# Note that this is different than the standard CRC-8 polynomial, because the
//# standard CRC-8 polynomial is not particularly good.
//# nibble lookup table for (x^8 + x^5 + x^3 + x^2 + x + 1)
#if 0
void genTable(U8 const generator)	
	::std::cout << "const FU8 Crc8_Config::lookupTable[16] = {\n\t"; 
	forInc(Int, dividend, 0, 16) {
		U8 currbyte = dividend;
		
		forInc(FU8, bit, 0, 8) {
			if((currbyte & 0x80) != 0) {
				currbyte <<= 1;
				currbyte ^= generator;
			}
			else {
				currbyte <<= 1;
			}
		}
		::std::cout << currbyte << ", ";
//		lookup[dividend] = currbyte;
	}
	::std::cout << "\b\b\n};" << ::std::endl; 
}
int main() {
	genTable(B0010_1111);
	return 0;
}
#endif // 0
const U8 Crc8_Config::lookupTable[16] = {
	0, 47, 94, 113, 188, 147, 226, 205, 87, 120, 9, 38, 235, 196, 181, 154, 
};

//# 12us/add
typedef Crc8_Bit<Crc8_Config> Crc;
//# 6us/add. 2x faster for cost of 31 flash bytes 
//typedef Crc8_Nibble_OneTable<Crc8_Config> Crc;

struct MinMaxRollingBinaryTreeFinder_Store {
	struct {
		Math::MinMax<TempK10_6> weekMinMaxTempCircularBuffer_data[7];
		Math::MinMax<TempK10_6> monthMinMaxTempCircularBuffer_data[4];
		U8 weekMinMaxTempCircularBuffer_index;
		U8 monthMinMaxTempCircularBuffer_index;
		U8 id;
	} data;
	Crc::CrcValue crc;
	
	void init() {
		this->data.weekMinMaxTempCircularBuffer_index = -1; 
		this->data.monthMinMaxTempCircularBuffer_index = -1;
	}
	
	enum Read_ErrorCode {
		Read_ErrorCode_ok, 
		Read_ErrorCode_invalidCrc, 
		Read_ErrorCode_corruptedCopy, 
	};
	Crc::CrcValue calcCrc() const {
		Crc crc;
		crc.init();
		forInc(FU8, i, 0, sizeof(this->data)) {
			crc.add(reinterpret_cast<U8 const*>(&(this->data))[i]);
		}
		return crc.getResult();
	}
	inline Bool isValid() const {
		return this->crc == this->calcCrc();
	}

	static Int findLastStoreIndexByBitSet(MinMaxRollingBinaryTreeFinder_Store const eepromDests[2], FU8 validIndecesBitSet);
	static FU8 srcsToBitSet(MinMaxRollingBinaryTreeFinder_Store const eepromDests[2]);
	static Int findLastStoreIndex(MinMaxRollingBinaryTreeFinder_Store const eepromDests[2]) {
		return findLastStoreIndexByBitSet(eepromDests, srcsToBitSet(eepromDests));
	}
	void updateExternalData();
	void readToRamRaw(MinMaxRollingBinaryTreeFinder_Store const& eepromSrc);
	void readToRam(MinMaxRollingBinaryTreeFinder_Store const& eepromSrc);
	Read_ErrorCode readBackupToRam_noBackup(MinMaxRollingBinaryTreeFinder_Store const& eepromSrc);
	Read_ErrorCode readBackupToRam_switch(MinMaxRollingBinaryTreeFinder_Store const eepromSrcs[2]);
	Read_ErrorCode readBackupToRam_duplicate(MinMaxRollingBinaryTreeFinder_Store const eepromSrcs[2]);
	void updateFromExternalData();
	void writeToEepromRaw(MinMaxRollingBinaryTreeFinder_Store& eepromDest);
	void writeBackupToEeprom_noBackup(MinMaxRollingBinaryTreeFinder_Store& eepromDest);
	void writeBackupToEeprom_duplicate(MinMaxRollingBinaryTreeFinder_Store eepromDests[2]);
	void writeBackupToEeprom_switch(MinMaxRollingBinaryTreeFinder_Store eepromDests[2]);
};

//# 4 bytes aligned
static_assert_test(sizeof(MinMaxRollingBinaryTreeFinder_Store), x % 4 == 0);

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
	//# TODO automatic calculation of gap
	U8 BGA__UNIQUE_NAME[2]; //# align gap to 4 bytes
	MinMaxRollingBinaryTreeFinder_Store MinMaxRollingBinaryTreeFinder_stores[2]; 
};

static_assert_test(offsetof(Settings, MinMaxRollingBinaryTreeFinder_stores), x % 4 == 0);

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
Settings& settingsRw = ((Settings*)(&defaultSettings))[0];

FU16 rrToRh(FU16 adc, FU16 rL) {
	return FU32(rL) * (Adc_maxValueOversampled - adc) / adc;
//	return rL * (Adc_maxValue / adc - 1);
}
FU16 rrToRl(FU16 adc, FU16 rH) {
	return FU32(rH) * adc / (Adc_maxValueOversampled - adc);
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



enum { lastTemp_notFilledMagicNumber = 0x7FFE };
FTempK10_6 lastTemp = lastTemp_notFilledMagicNumber;


FU16 ticksCountLive = 0;


void sysClockTask() {
	ticksCountLive += 1;
}

FU8 display_index = 0;
void displayTask() {
	FU16 ticksCount = ticksCountLive;

	#if 1
	display.updateManual(display_index);
	cycleInc(display_index, 6);
	#endif // 1
}

//namespace MeasureThread {

	struct TaskArgs {

	};

	typedef Scheduler::Stack<TaskArgs, FU8> MeasureThread_Scheduler;

	extern MeasureThread_Scheduler scheduler;

	struct A {
		static inline void convertAndDisplayRawTemp(FTempK10_6 x, FU8* dest) {
			displayTemp(User_convertTemperature((FI32(x) * 10) >> 6), dest);
		}
	};


	enum { readAdcTask_fetchPeriod = msToTicksCount(Config::readAdcTask_fetchPeriod_ms) };
	FU16 adcTicksCount = 0;

	RunningAvg<Adc_Value[Config::AdcUser_adcsSize], Config::AdcUser_RunningAvgSumType> tempAdcRunningAvg;
	Bool tempAdc_isHwError;

	void readAdcTask(TaskArgs taskArgs);
	void readAdcTask_push() {
		if(readAdcTask_fetchPeriod <= (adcTicksCount += 1)) {
			adcTicksCount = 0;
			scheduler.push(readAdcTask);
		};
	}
	void readAdcTask(TaskArgs taskArgs) {
		Config::tempAdcGpioVccPort.on();
		Adc_Value t = 0;
		forInc(FU8, i, 0, (1 << (2 * Config::AdcUser_oversampleExtraBitsCount))) {
			t += Config::tempAdcGpioPort.readSync();
		}
		t >>= Config::AdcUser_oversampleExtraBitsCount;
		Config::tempAdcGpioVccPort.off();

		if(AdcUser_checkWrongValue(t)) {
			tempAdcRunningAvg.add(t);
			tempAdc_isHwError = false;
		}
		else {
			tempAdc_isHwError = true;
		}
	}


	FU16 displayTicksCount = 0;
	void renderTempTask(TaskArgs taskArgs);
	void renderTempTask_push() {
		if(settings.displayUpdatePeriod <= (displayTicksCount += 1) ) {
			displayTicksCount = 0;
			scheduler.push(renderTempTask);
		};
	}
	void renderTempTask(TaskArgs taskArgs) {
		if(tempAdc_isHwError) {
			displayHwError(&(display.displayChars[Config::tempDisplayCharIndex]));
		}
		else if(!settings.tempAdcFix.isValid()) {
			displaySoftError(UserError_badEeprom, &(display.displayChars[Config::tempDisplayCharIndex]));
		}
		else {
			FU16 adcAvg = tempAdcRunningAvg.computeAvg();
			FU16 rT = rrToRl(adcAvg, settings.rDiv);
			FTempK10_6 temp = settings.tempAdcFix.m_ntcThermistor.convert(rT);

			if(lastTemp == lastTemp_notFilledMagicNumber || settings.tempHysteresis < Math_abs(temp - lastTemp)) {
				lastTemp = temp;
				A::convertAndDisplayRawTemp(temp, &(display.displayChars[Config::tempDisplayCharIndex]));
			};
		}
	}

	void adcDump() {
		displayDecrimal6(tempAdcRunningAvg.computeAvg(), &(display.displayChars[0]));
	}

	enum {
		HLDisplayData_h1 = 0,
		HLDisplayData_l1,
		HLDisplayData_h7,
		HLDisplayData_l7,
		HLDisplayData_h30,
		HLDisplayData_l30,
	};

	const FTempK10_6 HLDisplayData_notFilledMagic = FTempK10_6(0x8000);


	struct HLDisplayData {
		const char name[3];
		FTempK10_6 temp;
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

	void renderHLDisplayDataTask(TaskArgs taskArgs);
	void renderHLDisplayDataTask_push() {
		if(HLDisplayData_isNotFilled()) {
			//# display nothing
		}
		else {
			scheduler.push(renderHLDisplayDataTask);
		}
	}
	void renderHLDisplayDataTask(TaskArgs taskArgs) {
		if(0 == hlDisplayData_ticksCount) {
			memcpy(&(display.displayChars[Config::hlDisplayDataCharIndex]), hlDisplayData[hlDisplayData_index].name, sizeof(hlDisplayData[hlDisplayData_index].name));
		}
		else if(settings.hlDisplayData.nameDisplayEndTime == hlDisplayData_ticksCount) {
			A::convertAndDisplayRawTemp(hlDisplayData[hlDisplayData_index].temp, &(display.displayChars[Config::hlDisplayDataCharIndex]));
		}
		else if(settings.hlDisplayData.valueDisplayEndTime == hlDisplayData_ticksCount) {
			cycleInc(hlDisplayData_index, arraySize(hlDisplayData));
			// cycleInc(hlDisplayData_index, 2); //# do not show week and month min/max until it will be filled
			hlDisplayData_ticksCount = -1;
		}
		hlDisplayData_ticksCount += 1;
	}


	struct MinMaxRollingBinaryTreeFinder_Config {
		enum {
			levelMax = Config::MinMaxRollingBinaryTreeFinder_Config_levelMax,
			levelValuesSizeLog2 = Config::MinMaxRollingBinaryTreeFinder_Config_levelValuesSizeLog2,
		};

		typedef FU16 Index;
		typedef FU8 SubIndex;
		typedef FU8 Level;
		typedef TempK10_6 DataValue;
	};

	enum {
		MinMaxRollingBinaryTreeFinder_pushInterval = (
			#if 1
				msToTicksCount(Config::MinMaxRollingBinaryTreeFinder_pushInterval_ms)
			#else
				0 //# for debug
			#endif
		),
		MinMaxRollingBinaryTreeFinder_forceUpdateLog2 = Config::MinMaxRollingBinaryTreeFinder_forceUpdateLog2,
	};

	FU16 MinMaxRollingBinaryTreeFinder_ticksCount = 0;

	typedef Math::MinMaxRollingBinaryTreeFinder<MinMaxRollingBinaryTreeFinder_Config> MinMaxRollingBinaryTreeFinder;
	typedef MinMaxRollingBinaryTreeFinder::MinMaxD MinMaxRollingBinaryTreeFinder_MinMaxD;
	MinMaxRollingBinaryTreeFinder minMaxRollingBinaryTreeFinder;

	MinMaxRollingBinaryTreeFinder_Store MinMaxRollingBinaryTreeFinder_store;
	
	CircularBuffer_Dynamic<MinMaxRollingBinaryTreeFinder_MinMaxD, FU8> weekMinMaxTempCircularBuffer(MinMaxRollingBinaryTreeFinder_store.data.weekMinMaxTempCircularBuffer_data, arraySize(MinMaxRollingBinaryTreeFinder_store.data.weekMinMaxTempCircularBuffer_data));
	CircularBuffer_Dynamic<MinMaxRollingBinaryTreeFinder_MinMaxD, FU8> monthMinMaxTempCircularBuffer(MinMaxRollingBinaryTreeFinder_store.data.monthMinMaxTempCircularBuffer_data, arraySize(MinMaxRollingBinaryTreeFinder_store.data.monthMinMaxTempCircularBuffer_data));

	void MinMaxRollingBinaryTreeFinder_Store::updateExternalData() {
		weekMinMaxTempCircularBuffer.index = this->data.weekMinMaxTempCircularBuffer_index;
		monthMinMaxTempCircularBuffer.index = this->data.monthMinMaxTempCircularBuffer_index;
	}
	void MinMaxRollingBinaryTreeFinder_Store::readToRamRaw(MinMaxRollingBinaryTreeFinder_Store const& eepromSrc) {
		Eeprom_read(*this, eepromSrc);
	}
	void MinMaxRollingBinaryTreeFinder_Store::readToRam(MinMaxRollingBinaryTreeFinder_Store const& eepromSrc) {
		Eeprom_read(*this, eepromSrc);
		this->updateExternalData();
	}
	MinMaxRollingBinaryTreeFinder_Store::Read_ErrorCode MinMaxRollingBinaryTreeFinder_Store::readBackupToRam_noBackup(MinMaxRollingBinaryTreeFinder_Store const& eepromSrc) {
		this->readToRam(eepromSrc);
		if(this->calcCrc() != this->crc) {
			return Read_ErrorCode_invalidCrc;
		};
		return Read_ErrorCode_ok;
	}
	MinMaxRollingBinaryTreeFinder_Store::Read_ErrorCode MinMaxRollingBinaryTreeFinder_Store::readBackupToRam_duplicate(MinMaxRollingBinaryTreeFinder_Store const eepromSrcs[2]) {
		FU8 validBitSet = srcsToBitSet(eepromSrcs);
		FI8 lastIndex = this->findLastStoreIndexByBitSet(eepromSrcs, validBitSet);
		
		if(lastIndex < 0) {
			return Read_ErrorCode_invalidCrc;
		}
		else {
			this->readToRam(eepromSrcs[lastIndex]);
			return (validBitSet == 3) ? Read_ErrorCode_ok : Read_ErrorCode_corruptedCopy;
		}
	}
	MinMaxRollingBinaryTreeFinder_Store::Read_ErrorCode MinMaxRollingBinaryTreeFinder_Store::readBackupToRam_switch(MinMaxRollingBinaryTreeFinder_Store const eepromSrcs[2]) {
		FU8 validBitSet = srcsToBitSet(eepromSrcs);
		FI8 lastIndex = this->findLastStoreIndexByBitSet(eepromSrcs, validBitSet);
		
		if(lastIndex < 0) {
			return Read_ErrorCode_invalidCrc;
		}
		else {
			this->readToRam(eepromSrcs[lastIndex]);
			return (validBitSet == 3) ? Read_ErrorCode_ok : Read_ErrorCode_corruptedCopy;
		}
	}
	inline void MinMaxRollingBinaryTreeFinder_Store::updateFromExternalData() {
		this->data.weekMinMaxTempCircularBuffer_index = weekMinMaxTempCircularBuffer.index; 
		this->data.monthMinMaxTempCircularBuffer_index = monthMinMaxTempCircularBuffer.index;
		this->crc = this->calcCrc();
	}
	inline void MinMaxRollingBinaryTreeFinder_Store::writeToEepromRaw(MinMaxRollingBinaryTreeFinder_Store& eepromDest) {
		//# change it to { Eeprom_write1 } if out of flash memory. -192 bytes but 4x slower write  
		Eeprom_writeFast32(eepromDest, *this);
//		Eeprom_write(eepromDest, *this);
	}

	void MinMaxRollingBinaryTreeFinder_Store::writeBackupToEeprom_noBackup(MinMaxRollingBinaryTreeFinder_Store& eepromDest) {
		this->updateFromExternalData();
		this->writeToEepromRaw(eepromDest);
	}
	void MinMaxRollingBinaryTreeFinder_Store::writeBackupToEeprom_duplicate(MinMaxRollingBinaryTreeFinder_Store eepromDests[2]) {
		this->updateFromExternalData();
		forInc(FU8, i, 0, 2) {
			this->writeToEepromRaw(eepromDests[i]);
		}
	}
	FU8 MinMaxRollingBinaryTreeFinder_Store::srcsToBitSet(MinMaxRollingBinaryTreeFinder_Store const eepromDests[2]) {
		return (eepromDests[0].isValid() ? 1 : 0) | ((eepromDests[1].isValid() ? 1 : 0) << 1); 
	}
	Int MinMaxRollingBinaryTreeFinder_Store::findLastStoreIndexByBitSet(MinMaxRollingBinaryTreeFinder_Store const eepromDests[2], FU8 validIndecesBitSet) {
		switch(validIndecesBitSet) {
		  case(0): {
				return -1;
			} break;
			case(1): {
				return 0;
			} break;
			case(2): {
				return 1;
		 } break;
		  case(3): {
				if(FU8(eepromDests[1].data.id - eepromDests[0].data.id) < FU8(eepromDests[0].data.id - eepromDests[1].data.id)) {
					return 0;
				}
				else {
					return 1;
				}
			} break;
			default: {
				return -2;
			}
		}
	}
	void MinMaxRollingBinaryTreeFinder_Store::writeBackupToEeprom_switch(MinMaxRollingBinaryTreeFinder_Store eepromDests[2]) {
		FU8 nextIndex;
		FU8 nextId;
		FI8 lastIndex = this->findLastStoreIndex(eepromDests);
		
		if(lastIndex < 0) {
			nextId = 0;
			nextIndex = 0;
		}
		else {
			FU8 lastId = eepromDests[lastIndex].data.id;
			nextId = lastId + 1;
			nextIndex = lastIndex ^ 1; //# 0 -> 1, 1 -> 0
		}
		this->data.id = nextId; 
		this->writeBackupToEeprom_noBackup(eepromDests[nextIndex]);
	}
	
	

	MinMaxRollingBinaryTreeFinder_MinMaxD CircularBuffer_addMinMax(const MinMaxRollingBinaryTreeFinder_MinMaxD& minMax, CircularBuffer_Dynamic<MinMaxRollingBinaryTreeFinder_MinMaxD, FU8>& circularBuffer) {
		if(circularBuffer.initAndPrefill(minMax)) {
			return minMax;
		}
		else {
			circularBuffer.add(minMax);

			return MinMaxRollingBinaryTreeFinder_MinMaxD::fromArray(circularBuffer.data, circularBuffer.size);
		}
	}

	void pushMinMaxRollingBinaryTreeFinderTask(TaskArgs taskArgs);
	void pushMinMaxRollingBinaryTreeFinderTask_forceDispatch() {
		if(lastTemp != lastTemp_notFilledMagicNumber && MinMaxRollingBinaryTreeFinder_pushInterval <= (MinMaxRollingBinaryTreeFinder_ticksCount += 1)) {
			MinMaxRollingBinaryTreeFinder_ticksCount = 0;
			pushMinMaxRollingBinaryTreeFinderTask(TaskArgs());
		};
	}
	void pushMinMaxRollingBinaryTreeFinderTask(TaskArgs taskArgs) {
		MinMaxRollingBinaryTreeFinder_MinMaxD tempMinMax = minMaxRollingBinaryTreeFinder.addValue(lastTemp, minMaxRollingBinaryTreeFinder.levelMax - MinMaxRollingBinaryTreeFinder_forceUpdateLog2);
		if(minMaxRollingBinaryTreeFinder.isCarry(MinMaxRollingBinaryTreeFinder_forceUpdateLog2)) {
			#if 1
			if(HLDisplayData_isNotFilled() || minMaxRollingBinaryTreeFinder.isCarry(0)) {
				MinMaxRollingBinaryTreeFinder_MinMaxD weekMinMaxTemp = CircularBuffer_addMinMax(tempMinMax, weekMinMaxTempCircularBuffer);
				hlDisplayData[HLDisplayData_h7].temp = weekMinMaxTemp.max;
				hlDisplayData[HLDisplayData_l7].temp = weekMinMaxTemp.min;

				monthMinMaxTempCircularBuffer.initAndPrefill(weekMinMaxTemp);

				if(weekMinMaxTempCircularBuffer.isCarry()) {
					monthMinMaxTempCircularBuffer.cycleIndex();
				};
				monthMinMaxTempCircularBuffer.setCurrent(weekMinMaxTemp);

				MinMaxRollingBinaryTreeFinder_MinMaxD monthMinMaxTemp = MinMaxRollingBinaryTreeFinder_MinMaxD::fromArray(monthMinMaxTempCircularBuffer.data, monthMinMaxTempCircularBuffer.size);

				hlDisplayData[HLDisplayData_h30].temp = monthMinMaxTemp.max;
				hlDisplayData[HLDisplayData_l30].temp = monthMinMaxTemp.min;

				MinMaxRollingBinaryTreeFinder_store.CONFIG__WRITE_BACKUP_TO_EEPROM(settingsRw.MinMaxRollingBinaryTreeFinder_stores);
			};
			#endif

			hlDisplayData[HLDisplayData_h1].temp = tempMinMax.max;
			hlDisplayData[HLDisplayData_l1].temp = tempMinMax.min;
		};
	}

	void debugHeartBeatTask(TaskArgs taskArgs);
	void debugHeartBeatTask_push() {
		if(0) debug {
			scheduler.push(debugHeartBeatTask);
		}
	}
	void debugHeartBeatTask(TaskArgs taskArgs) {
		FU16 ticksCount = ticksCountLive;

		if((ticksCount & bitsCountToMask(7)) == 0) {
			display.displayChars[0] ^= _7SegmentsFont::dot;
		};
	}

	enum { MeasureThread_maxTaskDispatchesPerTick = 2 };
	MeasureThread_Scheduler::Task tasks[] = {
		readAdcTask,
		renderTempTask,
		renderHLDisplayDataTask,
		#ifndef NDEBUG
			debugHeartBeatTask,
		#endif
	};
	MeasureThread_Scheduler scheduler(tasks, arraySize(tasks));
//}

void timerThread(Timer& timer) {
	timer.clearPendingInterrupt();

	displayTask();
	sysClockTask();

//	using namespace MeasureThread;

	FU16 ticksCount = ticksCountLive;

	#if 0
		displayDecrimal(ticksCount, &(display.displayChars[3]));
		return;
	#endif // 1

	readAdcTask_push();
	if(isDebugMode) {
		adcDump();
	}
	else {
		renderTempTask_push();
		renderHLDisplayDataTask_push();
		pushMinMaxRollingBinaryTreeFinderTask_forceDispatch();
		
	}
	if(0) debug debugHeartBeatTask_push();

	while(!scheduler.isEmpty() && !timer.hasPendingInterrupt()) {
		scheduler.dispatch(TaskArgs());
	}

	debug if(timer.hasPendingInterrupt()) {
		APP__DEBUG__WRITE(F, U, L);
	};
}

void Clock_setCpuFullSpeed() {
	using namespace ::STM8S_StdPeriph_Lib;
	CLK->CKDIVR = 0;
}

void Hw_enable() {
	using namespace ::STM8S_StdPeriph_Lib;
	//# CLK->PCKENR* by default has all bits set. We need only enable clocking for used peripheral. So direct assignment
	CLK->PCKENR1 = (0 
//		| CLK_PCKENR1_TIM2 
		| CLK_PCKENR1_TIM4
	);
	CLK->PCKENR2 = (0 
		| CLK_PCKENR2_ADC
	);
}


inline FU16 getServiceModePinShortageTime_ms() {
	
	FU16 time = 0;
	Config::debugEnableTest.m_gndPin.off();
	delay_us(10);
	//# wait until release
	while(Config::debugEnableTest.m_pullUpPin.read() == 0) {
		delay_ms(1);
		time += 1;
	}
	return time;
} 

void main() {

	Clock_setCpuFullSpeed();
	Hw_enable();
	
	FU8 errorCode = 0;
	
	#if 1
	Config::debugEnableTest.m_gndPin.init();
	Config::debugEnableTest.m_pullUpPin.init();
	FU16 serviceModePinShortageTime_ms = getServiceModePinShortageTime_ms();
	if(serviceModePinShortageTime_ms == 0) {
		//# normal mode
	}
	else if(Config::MinMaxRollingBinaryTreeFinder_resetEepromDataHoldDebugButtonDelayMin_ms <= serviceModePinShortageTime_ms) {
		MinMaxRollingBinaryTreeFinder_store.init();
		MinMaxRollingBinaryTreeFinder_store.CONFIG__WRITE_BACKUP_TO_EEPROM(settingsRw.MinMaxRollingBinaryTreeFinder_stores);
		errorCode = _7SegmentsFont::C;
	}
	else {
		isDebugMode = true;
	};
	
	switch(MinMaxRollingBinaryTreeFinder_store.CONFIG__READ_BACKUP_TO_RAM(settingsRw.MinMaxRollingBinaryTreeFinder_stores)) {
		case(MinMaxRollingBinaryTreeFinder_Store::Read_ErrorCode_invalidCrc): {
			MinMaxRollingBinaryTreeFinder_store.init();
			errorCode = _7SegmentsFont::E;
		} break;
		case(MinMaxRollingBinaryTreeFinder_Store::Read_ErrorCode_corruptedCopy): {
			errorCode = _7SegmentsFont::R;
		} break;
		default: {
			
		}
	}
	#endif // 0
	
	display.init();

	//# test data for display
	forInc(FU8, i, 0, 6) {
		display.displayChars[i] = (i < 3) ? _7SegmentsFont::digits[i] : 0;
	}
	

	if(errorCode != 0) {
		display.displayChars[0] = errorCode;
		display.updateManual(0);
		delay_ms(1500);
	};

//	displaySoftError(12, &(display.displayChars[0]));
//	displayTemp(FI10_6(4.2 * (1UL << 6)), &(display.displayChars[0]));
//	displayTemp(FI10_6(24.2 * (1UL << 6)), &(display.displayChars[0]));
//	displayTemp(FI10_6(124.2 * (1UL << 6)), &(display.displayChars[0]));
//?	displayDecrimal6(settings.displayUpdatePeriod, &(display.displayChars[0]));

	Adc_init();

	Config::tempAdcGpioPort.init();
	Config::tempAdcGpioVccPort.init();

	timer.init();
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
