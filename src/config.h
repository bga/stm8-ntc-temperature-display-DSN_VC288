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


namespace App { namespace Config {
	enum { 
		tempDisplayCharIndex = 0,  
		hlDisplayDataCharIndex = 3, 
	};

	enum {
		AdcUser_fetchSpeedPrescaler = 6,
		
		AdcUser_adcsSize = 32,
		
		AdcUser_minValue = 100,
		AdcUser_maxValue = 1000,
	};

	using namespace ::Bga::Mcu::Hal;
	
	struct Display {
		
		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 5> m_digit2CathodeGpioPort;
		Pin::PullHiZ<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 6> m_digit1CathodeGpioPort;
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
	
	Pin::AdcWithEnable<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 3, 4, 24 /* us */> tempAdcGpioPort;
	Pin::PushPull<(::STM8S_StdPeriph_Lib::GPIOD_BaseAddress), 2> tempAdcGpioVccPort;
} } //# namespace
