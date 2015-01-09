// STM32F031 Support
#pragma once

#include "utils.h"
#include "gpio.h"
#include "dma.h"


namespace Mcucpp
{
	namespace Adcs
	{
		enum BaseConfig
		{
			Discontinuous = ADC_CFGR1_DISCEN,
			AutoOff = ADC_CFGR1_AUTOFF,
			Wait = ADC_CFGR1_WAIT,
			Continuous = ADC_CFGR1_CONT,
			OverrunPreserve = 0,
			OverrunOverwrite = ADC_CFGR1_OVRMOD,
			RightAlign = 0,
			LeftAlign = ADC_CFGR1_ALIGN,
			UpScan = 0,
			DownScan = ADC_CFGR1_SCANDIR,

			DefaultConfig = OverrunPreserve | RightAlign | UpScan
		};
		constexpr BaseConfig operator|(BaseConfig b1, BaseConfig b2)
		{
			return static_cast<BaseConfig>(uint32_t(b1) | uint32_t(b2));
		}

		enum class Tsample
		{
			_1c5, _7c5, _13c5, _28c5, _41c5, _55c5, _71c5, _239c5
		};
		constexpr Tsample operator "" _cycles(long double c)
		{
			return c == 1.5 ? Tsample::_1c5 :
				c == 7.5 ? Tsample::_7c5 :
				c == 13.5 ? Tsample::_13c5 :
				c == 28.5 ? Tsample::_28c5 :
				c == 41.5 ? Tsample::_41c5 :
				c == 55.5 ? Tsample::_55c5 :
				c == 71.5 ? Tsample::_71c5 :
				c == 239.5 ? Tsample::_239c5 : Tsample(__UINT32_MAX__);
		}

		enum class Clock : uint32_t			//TODO: Set unsigned enums in compiler options
		{
			Hsi14 = 0,
			PclkDiv2 = ADC_CFGR2_CKMODE_0,
			PclkDiv4 = ADC_CFGR2_CKMODE_1
		};

		enum class Ch
		{
			Pa0_0 = ADC_CHSELR_CHSEL0,
			Pa1_1 = ADC_CHSELR_CHSEL1,
			Pa2_2 = ADC_CHSELR_CHSEL2,
			Pa3_3 = ADC_CHSELR_CHSEL3,
			Pa4_4 = ADC_CHSELR_CHSEL4,
			Pa5_5 = ADC_CHSELR_CHSEL5,
			Pa6_6 = ADC_CHSELR_CHSEL6,
			Pa7_7 = ADC_CHSELR_CHSEL7,
			Pb0_8 = ADC_CHSELR_CHSEL8,
			Pb1_9 = ADC_CHSELR_CHSEL9,
			TSense = ADC_CHSELR_CHSEL16,
			VRefSense = ADC_CHSELR_CHSEL17
		};
		constexpr Ch operator|(Ch c1, Ch c2)
		{
			return static_cast<Ch>((uint32_t)c1 | (uint32_t)c2);
		}
		constexpr Ch operator "" _ch(unsigned long long int c)
		{
			return static_cast<Ch>(c);
		}

// Not necessary init options

		namespace Private {
			template<typename...>
			struct AdcInitStruct;
			template<typename First, typename... Rest>
			struct AdcInitStruct<First, Rest...>
			{
				using RestStuff = AdcInitStruct<Rest...>;
				constexpr static uint32_t CFGR1 = First::CFGR1 | RestStuff::CFGR1;
				template<typename Adc>
				struct Wrapper
				{
					static void Apply()
					{
						First::template Wrapper<Adc>::Apply();
						RestStuff::template Wrapper<Adc>::Apply();
					}
				};
			};
			template<> struct AdcInitStruct<>
			{
				constexpr static uint32_t CFGR1 = 0;
				template<typename> struct Wrapper
				{
					static void Apply() { }
				};
			};
			using InitStructBase = AdcInitStruct<>;
		}

		template<Ch channels, bool andSelect = false>
		struct InitChannels : public Private::InitStructBase
		{
			template<typename Adc>
			struct Wrapper : public Adc
			{
				static void Apply()
				{
					constexpr uint32_t ch = static_cast<uint32_t>(channels);
					constexpr uint32_t sense = (ch & uint32_t(Ch::TSense) ? ADC_CCR_TSEN : 0) |
												(ch & uint32_t(Ch::VRefSense) ? ADC_CCR_VREFEN : 0);
					if(ch & 0xFF) Porta::SetConfig<ch & 0xFF, Gpio::Input, Gpio::Analog>();
					if((ch >> 8UL) & 0xFF) Portb::SetConfig<(ch >> 8UL) & 0xFF, Gpio::Input, Gpio::Analog>();
					if(sense) ADC->CCR = sense;
					if(andSelect) Adc::template SelectChannels<channels>();
				}
			};
		};

		template<uint32_t LowLevel, uint32_t HighLevel, Ch channel = Ch(__UINT32_MAX__)>
		struct EnableAnalogWatchdog : public Private::InitStructBase
		{
			static constexpr uint32_t CFGR1 = ADC_CFGR1_AWDEN | (uint32_t(channel) != __UINT32_MAX__ ?
					ADC_CFGR1_AWDSGL | Mask2Position<uint32_t(channel)>::value << 26UL : 0);
			template<typename Adc>
			struct Wrapper : public Adc
			{
				static void Apply()
				{
					Adc::Regs()->TR = LowLevel | HighLevel << 16UL;
				}
			};
		};

		enum class TrigEdge
		{
			NoTrigger, Rising, Falling, Both
		};
		enum class TrigEvent
		{
			Tim1_Trgo, Tim1_Cc4, Tim2_Trgo, Tim3_Trgo, Tim15_Trgo
		};
		template<TrigEdge tedge, TrigEvent tevent>
		struct EnableExtTrigger : public Private::InitStructBase
		{
			static constexpr uint32_t CFGR1 = static_cast<uint32_t>(tedge) << 10UL | static_cast<uint32_t>(tevent) << 6UL;
		};

		enum class DmaMode
		{
			OneShot = 0,
			Circular = ADC_CFGR1_DMACFG
		};
		template<DmaMode mode = DmaMode::OneShot>
		struct EnableDma : public Private::InitStructBase
		{
			static constexpr uint32_t CFGR1 = ADC_CFGR1_DMAEN | static_cast<uint32_t>(mode);
		};

//---------------------------------------------
	namespace Private
	{
		template<uint32_t BaseAddr>
		class Adc
		{
		protected:
			using Self = Adc<BaseAddr>;
			constexpr static ADC_TypeDef* Regs()
			{
				return reinterpret_cast<ADC_TypeDef*>(BaseAddr);
			}
		public:
			using Dma = Dmas::DmaCh1;
			template<BaseConfig conf = DefaultConfig, Clock clock = Clock::PclkDiv4, typename... Config>
			static void Init()
			{
				using InitStruct = Private::AdcInitStruct<Config...>;
		// Clock Enable
				RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
				if(clock == Clock::Hsi14)
				{
					RCC->CR2 |= RCC_CR2_HSI14ON;
					while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
				}
				Regs()->CFGR2 = static_cast<uint32_t>(clock);
		//
				Calibrate();
		// There can be resolution configuration

		// Enable ADC
				Regs()->CR = ADC_CR_ADEN;
				while(!(Regs()->ISR & ADC_ISR_ADRDY));
		// Rest configuration
				if(conf | InitStruct::CFGR1) Regs()->CFGR1 = conf | InitStruct::CFGR1;
				InitStruct::template Wrapper<Self>::Apply();
			}
			static void Calibrate()
			{
				Disable();
				Regs()->CR = ADC_CR_ADCAL;
				delay_ns<500>();
				while(Regs()->CR & ADC_CR_ADCAL);
			}
			static void Disable()
			{
				uint32_t CR = Regs()->CR;
				if((CR & ADC_CR_ADEN) && !(CR & ADC_CR_ADDIS))
				{
					Stop();
					Regs()->CR = ADC_CR_ADDIS;
				}
				while(Regs()->CR & ADC_CR_ADEN);
			}
			static void Stop()
			{
				if(Regs()->CR & ADC_CR_ADSTART) Regs()->CR = ADC_CR_ADSTP;
				while(Regs()->CR & ADC_CR_ADSTP);
			}
			template<Ch channels>
			static void SelectChannels()
			{
				Regs()->CHSELR = static_cast<uint32_t>(channels);
			}
			static void SelectChannels(Ch channels)
			{
				Regs()->CHSELR = static_cast<uint32_t>(channels);
			}
			template<Tsample t>
			static void SetTsample()
			{
				Regs()->SMPR = static_cast<uint32_t>(t);
			}
			static uint16_t GetSample()
			{
				Start();
				while(!(Regs()->ISR & ADC_ISR_EOC));
				return Regs()->DR;
			}

			static void Start()
			{
				Regs()->CR = ADC_CR_ADSTART;
			}

		};
	}//Private

	static inline uint16_t ConvertTemperature(uint16_t sample)
	{
		return ((110 - 30) * (sample - TS_CAL1))/(TS_CAL2 - TS_CAL1) + 30;
	}

	}//Adcs
	using Adc = Adcs::Private::Adc<ADC1_BASE>;
}
