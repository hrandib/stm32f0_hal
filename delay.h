// STM32F031 & STM32F051
#pragma once

#include "stm32f0xx.h"
#include <stdint.h>
namespace Mcucpp
{
	enum
	{	
		PlatformCyslesPerDelayLoop32 = 4,
	};
#if defined __GNUC__
//Cortex-M0 implementation
	__attribute__((always_inline))
	inline void __delay_loop(uint32_t delayLoops)
	{
		__asm__ __volatile__
			(
				"1: \n"
				"SUB %[delayLoops], #1 \n"
				"BGT 1b \n"
				: [delayLoops] "+l"(delayLoops)
			);
	}
#endif

	template<unsigned long ns, unsigned long CpuFreq = F_CPU>
	inline void delay_ns()
	{
		const unsigned long ns_corr = ns > uint64_t(7.0e9 / CpuFreq) ? ns - uint64_t(7.0e9 / CpuFreq) : 0;
		const unsigned long delayLoops32 = (unsigned long)(CpuFreq / (1.0e9 * PlatformCyslesPerDelayLoop32) * ns_corr);
		__delay_loop(delayLoops32);
	}

	template<unsigned long us, unsigned long CpuFreq = F_CPU>
	inline void delay_us()
	{
		const unsigned long delayLoops32 = (unsigned long)(F_CPU / (1.0e6 * PlatformCyslesPerDelayLoop32) * us);
		__delay_loop(delayLoops32);
	}

	template<unsigned long ms, unsigned long CpuFreq = F_CPU>
	inline void delay_ms()
	{
		const unsigned long delayLoops32 = (unsigned long)(CpuFreq / (1.0e3 * PlatformCyslesPerDelayLoop32) * ms);
		__delay_loop(delayLoops32);
	}

}
