// STM32F031 & STM32F051 Support
#pragma once

#include <type_traits>
#include "gpio.h"

namespace Mcucpp
{
	namespace Timers
	{
		typedef uint16_t Prescaler_t;
		typedef uint16_t Autoreload_t;

		enum Cfg
		{
			ARRbuffered = TIM_CR1_ARPE,
			UpCount = 0,
			DownCount = TIM_CR1_DIR,
			OnePulse = TIM_CR1_OPM
			//TODO: Write rest config flags
		};
		constexpr Cfg operator|(Cfg c1, Cfg c2)
		{ return static_cast<Cfg>(uint32_t(c1) | uint32_t(c2)); }

		enum ChannelNum
		{
			Ch1, Ch2, Ch3, Ch4, AllChannels
		};

		enum ChModeOutput
		{
			Output
		};

		enum ChCfgOut
		{
			Out_Default = 0,
			Out_Fast = TIM_CCMR1_OC1FE,
			Out_Preload = TIM_CCMR1_OC1PE,
			Out_Clear = TIM_CCMR1_OC1CE,
		};

		enum ActiveLevel
		{
			ActiveHigh,
			ActiveLow
		};

		enum ChCfgCompareMode
		{
			Out_Frozen,
			Out_ActiveOnMatch,
			Out_InactiveOnMatch,
			Out_Toggle,
			Out_ForceInactive,
			Out_ForceActive,
			
			// PWM mode 1 - In up counting, channel x is active as long as TIMx_CNT < TIMx_CCRx 
			// else inactive. In down counting, channel x is inactive(OCxREF = �0�) as long as
			// TIMx_CNT > TIMx_CCRx else active(OCxREF = �1�).
			Out_PwmMode1,

			// PWM mode 2 - In up counting, channel x is inactive as long as TIMx_CNT < TIMx_CCR1
			// else active. In down counting, channel x is active as long as TIMx_CNT>TIMx_CCRx else
			// inactive.
			Out_PwmMode2
		};

		enum ChModeInput
		{
			InputMode1 = 1,
			InputMode2,
			InputMode3,
		};

		enum ChannelCfgInFilter
		{
			In_NoFilter,
			In_Filt_n2,				// 0001: fSAMPLING= fMASTER, N = 2
			In_Filt_n4,				// 0010: fSAMPLING= fMASTER, N = 4
			In_Filt_n8,				// 0011: fSAMPLING= fMASTER, N = 8 
			In_Filt_div2_n6,		// 0100: fSAMPLING= fMASTER/2, N = 6
			In_Filt_div2_n8,		// 0101: fSAMPLING= fMASTER/4, N = 8
			In_Filt_div4_n6,		// 0110: fSAMPLING= fMASTER/8, N = 8
			In_Filt_div4_n8,		// 0111: fSAMPLING= fMASTER/8, N = 8
			In_Filt_div8_n6,		// 1000: fSAMPLING= fMASTER/8, N = 8
			In_Filt_div8_n8,		// 1001: fSAMPLING= fMASTER/8, N = 8
			In_Filt_div16_n5,		// 1010: fSAMPLING= fMASTER/16, N = 8
			In_Filt_div16_n6,		// 1011: fSAMPLING= fMASTER/16, N = 8
			In_Filt_div16_n8,		// 1100: fSAMPLING= fMASTER/16, N = 8
			In_Filt_div32_n5,		// 1101: fSAMPLING= fMASTER/32, N = 8
			In_Filt_div32_n6,		// 1110: fSAMPLING= fMASTER/32, N = 8
			In_Filt_div32_n8,		// 1111: fSAMPLING= fMASTER/32, N = 8
		};

		enum ChannelCfgInPrescaler
		{
			In_NoPresc = 0,			// 00: no prescaler, capture is done each time an edge is detected on the capture input
			In_Presc_2ev,			// 01: Capture is done once every 2 events
			In_Presc_4ev,			// 10: Capture is done once every 4 events
			In_Presc_8ev,			// 11: Capture is done once every 8 events
		};

		enum DmaRequests
		{
			UpdateReq,
			CC1Req,
			CC2Req,
			CC3Req,
			CC4Req,
			ComReq,
			TriggerReq
		};

		enum IRQs
		{
			UpdateIRQ,
			CC1IRQ,
			CC2IRQ,
			CC3IRQ,
			CC4IRQ,
			ComIRQ,
			TriggerIRQ,
			BreakIRQ
		};
		
		enum Events
		{
			UpdateEv,
			CC1Ev,
			CC2Ev,
			CC3Ev,
			CC4Ev,
			ComEv,
			TriggerEv,
			BreakEv,
			CC1OvercaptureEv = 9,
			CC2OvercaptureEv,
			CC3OvercaptureEv,
			CC4OvercaptureEv
		};

		enum MasterMode
		{
			MM_Reset,
			MM_Enable,
			MM_Update,
			MM_ComparePulse,
			MM_OC1REF,
			MM_OC2REF,
			MM_OC3REF,
			MM_OC4REF
		};

		enum SlaveCfg
		{
			//TODO: Implement
			Default,
			PolNonInverted = 0,

		};

		enum SlaveSource
		{
			Itr0, Itr1, Itr2, Itr3,
			Ti1EdgeDetect,
			Ti1, Ti2,
			ExtTrigger
		};

		enum SlaveMode
		{
			SM_Disabled,
			SM_Encoder1,
			SM_Encoder2,
			SM_Encoder3,
			SM_Reset,
			SM_Gated,
			SM_Trigger,
			SM_ExtClock
		};

		namespace Private
		{
			template<uint32_t BaseAddr>
			class Timer
			{
			protected:
				using DataT = typename std::conditional<BaseAddr == TIM2_BASE, uint32_t, uint16_t>::type;
				static TIM_TypeDef* Regs()
				{
					return reinterpret_cast<TIM_TypeDef*>(BaseAddr);
				}
			public:
				template<Cfg cfg, Prescaler_t presc = 0, Autoreload_t autoreload = 0>
				static void Init()
				{
					switch(BaseAddr)
					{
					case TIM1_BASE: RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
						break;
					case TIM2_BASE: RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
						break;
					case TIM3_BASE: RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
						break;
					case TIM14_BASE: RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
						break;
					case TIM16_BASE: RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
						break;
					case TIM17_BASE: RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
						break;				 			 
#ifdef STM32F051
					case TIM15_BASE: RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
						break;
					case TIM6_BASE: RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
						break;
#endif
					}
					Regs()->CR1 = cfg;
					if(presc) Regs()->PSC = presc - 1;
					if(autoreload) Regs()->ARR = autoreload - 1;
					Regs()->BDTR |= TIM_BDTR_MOE;
				}
				static void MasterModeSelect(MasterMode mode)
				{
					Regs()->CR2 = mode << 4UL;
				}
				static void SlaveModeSelect(SlaveSource src, SlaveMode mode, SlaveCfg cfg = Default)
				{
					Regs()->SMCR = src << 4UL | mode | cfg;
				}

				static void Enable()
				{
					Regs()->CR1 |= TIM_CR1_CEN;
				}
				static void Disable()
				{
					Regs()->CR1 &= ~TIM_CR1_CEN;
				}
				static void WriteAutoReload(DataT c)
				{
					Regs()->ARR = c - 1;
				}
				static void WriteRepeatCounter(uint8_t c)
				{
					Regs()->RCR = c - 1;
				}
				static void WritePrescaler(uint16_t presc)
				{
					if(presc) Regs()->PSC = presc - 1;
				}
				template<ChannelNum ch, ChModeInput mode,
						 ChannelCfgInFilter infilter = In_NoFilter,
						 ChannelCfgInPrescaler inpresc = In_NoPresc>
				static void ChannelInit()
				{
					enum{ cfg = infilter << 4UL | inpresc << 2UL | mode };
					if(ch == AllChannels)
					{
						Regs()->CCMR1 = Regs()->CCMR2 = cfg | cfg << 8UL;
					}
					else if(ch < 2) Regs()->CCMR1 |= (ch == Ch1) ? cfg : cfg << 8UL;
					else Regs()->CCMR2 |= (ch == Ch3) ? cfg : cfg << 8UL;
				}
				template<ChannelNum ch, ChModeOutput mode,
						 ChCfgCompareMode compcfg,
						 ChCfgOut cfg_add = Out_Default>
				static void ChannelInit()
				{
					enum{ cfg = compcfg << 4UL | cfg_add << 2UL };
					if(ch == AllChannels)
					{
						Regs()->CCMR1 = Regs()->CCMR2 = cfg | cfg << 8UL;
					}
					else if(ch < 2) Regs()->CCMR1 |= (ch == Ch1) ? cfg : cfg << 8UL;
					else Regs()->CCMR2 |= (ch == Ch3) ? cfg : cfg << 8UL;
				}

				template<ChannelNum ch, ActiveLevel alevel = ActiveHigh>
				static void ChannelEnable()
				{
					Regs()->CCER |= (1 | alevel << 1) << 4 * ch ;
				}

				template<ChannelNum Ch>
				static void WriteCompare(DataT c)
				{
					reinterpret_cast<volatile uint32_t*>(&Regs()->CCR1)[Ch] = c;
				}
				template<ChannelNum Ch>
				static DataT ReadCompare()
				{
					return reinterpret_cast<volatile uint32_t*>(&Regs()->CCR1)[Ch];
				}

				static volatile uint32_t& Ccr1()
				{
					return Regs()->CCR1;
				}
				static volatile uint32_t& Ccr2()
				{
					return Regs()->CCR2;
				}
				static volatile uint32_t& Ccr3()
				{
					return Regs()->CCR1;
				}
				static volatile uint32_t& Ccr4()
				{
					return Regs()->CCR1;
				}

				static DataT ReadCount()
				{
					return Regs()->CNT;
				}

				static void DmaRequestEnable(DmaRequests req)
				{
					Regs()->DIER |= 1 << (8UL + req);
				}
				static void IRQEnable(IRQs irq)
				{
					Regs()->DIER |= 1 << irq;
				}
				
				template<Events ev>
				static bool Event()
				{
					return Regs()->SR & (1 << ev);
				}
				template<Events ev>
				static void ClearEvent()
				{
					Regs()->SR &= ~uint16_t(1 << ev);
				}
				static bool Enabled()
				{
					return Regs()->CR1 & TIM_CR1_CEN;
				}
				static void Update()
				{
					Regs()->EGR |= TIM_EGR_UG;
				}
			};
		}//Private
	
	}
		typedef Timers::Private::Timer<TIM1_BASE> Tim1;
		typedef Timers::Private::Timer<TIM2_BASE> Tim2;
		typedef Timers::Private::Timer<TIM2_BASE> Tim3;
		typedef Timers::Private::Timer<TIM14_BASE> Tim14;
		typedef Timers::Private::Timer<TIM16_BASE> Tim16;
		typedef Timers::Private::Timer<TIM17_BASE> Tim17;
#ifdef STM32F051
	using Tim6 = Timers::Private::Timer<TIM6_BASE>;
	using Tim15 = Timers::Private::Timer<TIM15_BASE>;
#endif
}