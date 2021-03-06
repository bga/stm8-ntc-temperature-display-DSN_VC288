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

#pragma once


#define CONFIG__EEPROM_STORE_METHOD switch
#define CONFIG__WRITE_BACKUP_TO_EEPROM BGA__CONCAT(writeBackupToEeprom_, CONFIG__EEPROM_STORE_METHOD)
#define CONFIG__READ_BACKUP_TO_RAM BGA__CONCAT(readBackupToRam_, CONFIG__EEPROM_STORE_METHOD)

namespace App { namespace Config {
	enum {
		tempDisplayCharIndex = 0,
		hlDisplayDataCharIndex = 3,
	};

	typedef FU16 AdcUser_RunningAvgSumType;
	
	enum {
		AdcUser_adcsSize = 16,
		AdcUser_oversampleExtraBitsCount = 2,

		AdcUser_minValue = (100 << AdcUser_oversampleExtraBitsCount),
		AdcUser_maxValue = (1000 << AdcUser_oversampleExtraBitsCount),
	};

	enum {
		renderTempTask_freqAprox = 1600UL,
	};
	
	static_assert_lt(UIntMax(AdcUser_adcsSize) * (::Bga::Mcu::Hal::Adc_maxValue << AdcUser_oversampleExtraBitsCount), (UIntMax(1) << sizeof(AdcUser_RunningAvgSumType) * 8));

	enum {
		MinMaxRollingBinaryTreeFinder_dayPeriod_ms = 60UL * 60 * 24 * 1000,
		MinMaxRollingBinaryTreeFinder_Config_levelMax = 8,
		MinMaxRollingBinaryTreeFinder_Config_levelValuesSizeLog2 = 2,
		MinMaxRollingBinaryTreeFinder_pushInterval_ms = (MinMaxRollingBinaryTreeFinder_dayPeriod_ms >> (MinMaxRollingBinaryTreeFinder_Config_levelMax * MinMaxRollingBinaryTreeFinder_Config_levelValuesSizeLog2)),

		User_minPossibleDisplayUpdatePeriod_ms = 500,

		readAdcTask_fetchPeriod_ms = (
			BGA__MATH__MIN(User_minPossibleDisplayUpdatePeriod_ms, MinMaxRollingBinaryTreeFinder_pushInterval_ms)
			/ (AdcUser_adcsSize / 2) //# fill adc data at least half for each minmax data push
		),

		MinMaxRollingBinaryTreeFinder_forceUpdateLog2 = 4,
		MinMaxRollingBinaryTreeFinder_forceSaveLog2 = 3,

		MinMaxRollingBinaryTreeFinder_resetEepromDataHoldDebugButtonDelayMin_ms = 5000, 
	};

	using namespace ::Bga::Mcu::Hal;

	struct Display {

		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 3> m_digit2CathodeGpioPort;
		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 2> m_digit1CathodeGpioPort;
		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 4> m_digit0CathodeGpioPort;
		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOA_BaseAddress), 1> m_digit5CathodeGpioPort;
		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOB_BaseAddress), 4> m_digit4CathodeGpioPort;
		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOB_BaseAddress), 5> m_digit3CathodeGpioPort;


		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOC_BaseAddress), 6> m_digit0AnodeGpioPort;
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOC_BaseAddress), 7> m_digit1AnodeGpioPort;
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOC_BaseAddress), 3> m_digit2AnodeGpioPort;
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOC_BaseAddress), 4> m_digit3AnodeGpioPort;
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOA_BaseAddress), 3> m_digit4AnodeGpioPort;
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOA_BaseAddress), 2> m_digit5AnodeGpioPort;
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 1> m_digit6AnodeGpioPort;
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOC_BaseAddress), 5> m_digitDotAnodeGpioPort;
	};

	struct DebugEnableTest {
		Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOB_BaseAddress), 4> m_gndPin;
		Pin::PullUp<(::STM8S_StdPeriph_Lib::GPIOA_BaseAddress), 1> m_pullUpPin;
	} debugEnableTest;

	Pin::AdcWithEnable<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 6, 6, 24 /* us */> tempAdcGpioPort;
	Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 5> tempAdcGpioVccPort;
} } //# namespace
