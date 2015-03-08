//Tested on STM32F030 & STM32F031 & STM32F051
#pragma once

#include <type_traits>
#include "gpio.h"

namespace Mcucpp
{
	namespace Timers
	{
		typedef uint16_t Prescaler_t;
		typedef uint16_t Autoreload_t;
		static const bool NoIRQ = false;

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
			// else inactive. In down counting, channel x is inactive(OCxREF = ‘0’) as long as
			// TIMx_CNT > TIMx_CCRx else active(OCxREF = ’1’).
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
			UpdateIRQ = 1U,
			CC1IRQ = 1U << 1,
			CC2IRQ = 1U << 2,
			CC3IRQ = 1U << 3,
			CC4IRQ = 1U << 4,
			ComIRQ = 1U << 5,
			TriggerIRQ = 1U << 6,
			BreakIRQ = 1U << 7
		};
		constexpr IRQs operator|(IRQs c1, IRQs c2)
		{ return static_cast<IRQs>(uint32_t(c1) | uint32_t(c2)); }

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
				template<Cfg cfg, Prescaler_t presc = 0, Autoreload_t autoreload = 0, bool EnableNvic_ = true>
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
#if defined STM32F051 || defined STM32F030		//only for stm32f030x8/
					case TIM15_BASE: RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
						break;
					case TIM6_BASE: RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
						break;
#endif
					}
					Regs()->CR1 = cfg;
					if(presc) Regs()->PSC = presc - 1;
					if(autoreload) Regs()->ARR = autoreload;
					Regs()->BDTR |= TIM_BDTR_MOE;
					if(EnableNvic_) EnableNvic();
				}
				static void EnableNvic()
				{
					constexpr static IRQn irq =
#if defined STM32F031 || STM32F051
									BaseAddr == TIM2_BASE ? TIM2_IRQn :
									BaseAddr == TIM6_BASE ? TIM6_IRQn :
#endif
									BaseAddr == TIM3_BASE ? TIM3_IRQn :
									BaseAddr == TIM14_BASE ? TIM14_IRQn :
									BaseAddr == TIM15_BASE ? TIM15_IRQn :
									BaseAddr == TIM16_BASE ? TIM16_IRQn :
									BaseAddr == TIM17_BASE ? TIM17_IRQn :
									BaseAddr == TIM1_BASE ? TIM1_CC_IRQn : (IRQn)0xFF;
					if(irq != (IRQn)0xFF) NVIC_EnableIRQ(irq);
					if(irq == TIM1_CC_IRQn) NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
				}

				static void SetMasterMode(MasterMode mode)
				{
					Regs()->CR2 = mode << 4UL;
				}
				static void SetSlaveMode(SlaveSource src, SlaveMode mode, SlaveCfg cfg = Default)
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
					Regs()->ARR = c;
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
				static void InitChannel()
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
				static void InitChannel()
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
				static void EnableChannel()
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

				static DataT ReadCounter()
				{
					return Regs()->CNT;
				}
				static void Clear()
				{
					Regs()->CNT = 0;
				}

				static void EnableDmaRequest(DmaRequests req)
				{
					Regs()->DIER |= 1 << (8UL + req);
				}
				static void DisableDmaRequest(DmaRequests req)
				{
					Regs()->DIER &= ~(1 << (8UL + req));
				}
				static void EnableIRQ(IRQs irq)
				{
					Regs()->DIER |= irq;
				}
				static void DisableIRQ(IRQs irq)
				{
					Regs()->DIER &= ~irq;
				}

				static bool IsEvent(const Events ev)
				{
					return Regs()->SR & (1 << ev);
				}
				static void ClearEvent(Events ev)
				{
					Regs()->SR = ~uint16_t(1 << ev);
				}
				static bool IsEnabled()
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
		typedef Timers::Private::Timer<TIM3_BASE> Tim3;
		typedef Timers::Private::Timer<TIM14_BASE> Tim14;
		typedef Timers::Private::Timer<TIM16_BASE> Tim16;
		typedef Timers::Private::Timer<TIM17_BASE> Tim17;
#if defined STM32F051 || defined STM32F030		//only for stm32f030x8
	using Tim6 = Timers::Private::Timer<TIM6_BASE>;
	using Tim15 = Timers::Private::Timer<TIM15_BASE>;
#endif
	namespace Watchdog
	{
		enum Period
		{
			_64ms = 40,
			_128ms = 80,
			_256ms = 160,
			_512ms = 320,
			_1s = 640,
			_3s = 1875,
			_6s = 3800
		};

		inline static void Refresh()
		{
			IWDG->KR = 0xAAAA;
		}

		inline static void Enable(Period p)
		{
			IWDG->KR = 0xCCCC;
			IWDG->KR = 0x5555;
			IWDG->PR = 4;	//div 64
			IWDG->RLR = p;
			while(IWDG->SR)
				;
			IWDG->KR = 0xAAAA;
		}
	}
}
