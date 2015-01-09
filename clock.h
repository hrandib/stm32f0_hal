// STM32F031 & STM32F051
#pragma once

#include "gpio.h"

namespace Mcucpp
{
	namespace Mco
	{
		enum class Source
		{
			NoClock, Hsi14,	Lsi, Lse, Sysclk, Hsi8, Hse, PllDiv2
#if defined STM32F042 || defined STM32F072
			,Hsi48, Pll = (uint32_t)PllDiv2 | 0x80
		};
		enum class Div
		{
			_1, _2, _4, _8, _16, _32, _64, _128
#endif
		};

		//Output - Pa8
#if defined STM32F042 || defined STM32F072
		template<Source src, Div div = Div::_1>
		inline void Enable()
		{
			Pa8::SetConfig<Gpio::OutputFastest, Gpio::AltPushPull>();
			RCC->CFGR = (RCC->CFGR & ~(0xFF << 24UL)) | ((uint32_t)src | (uint32_t)div << 4) << 24UL;
		}
#else
		template<Source src>
		inline void Enable()
		{
			Pa8::SetConfig<Gpio::OutputFastest, Gpio::AltPushPull>();
			RCC->CFGR = (RCC->CFGR & ~(0xFF << 24UL)) | (uint32_t)src << 24UL;
		}
#endif
		inline void Disable()
		{
			Pa8::SetConfig<Gpio::Input, Gpio::PullDown>();
			RCC->CFGR &= ~(0xFF << 24UL);
		}

	}
}
