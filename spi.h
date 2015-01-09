#pragma once

#include "stm32f0xx.h"
#include <type_traits>
#include "utils.h"
//#include "pinlist.h"
#include "dma.h"

namespace Mcucpp
{
	namespace Spis
	{
		enum BaseConfig
		{

		//	---=== SPI CR1 ===---			
			
			BidiMode = SPI_CR1_BIDIMODE,
			BidiOE = SPI_CR1_BIDIOE,
			CrcEn = SPI_CR1_CRCEN,
			CrcNext = SPI_CR1_CRCNEXT,

			//This should not be changed when SPI Enabled
			RxOnly = SPI_CR1_RXONLY,
			SoftSlaveManage = SPI_CR1_SSM,
			SlaveSelect = SPI_CR1_SSI,
			LSBfirst = SPI_CR1_LSBFIRST,
			MasterCfg = SPI_CR1_MSTR,
			SlaveCfg = 0,
			CPOL_0_Idle = 0,
			CPOL_1_Idle = SPI_CR1_CPOL,
			CPHA_0 = 0,
			CPHA_1 = SPI_CR1_CPHA,
		//	---=== SPI CR2 ===---			
			DmaTxEnable = SPI_CR2_TXDMAEN << 16UL,
			DmaRxEnable = SPI_CR2_RXDMAEN << 16UL,
			SSOE = SPI_CR2_SSOE << 16UL,

			//Master, no NSS 
			DefaultCfg = SoftSlaveManage | SlaveSelect | MasterCfg
		};
		constexpr BaseConfig operator|(BaseConfig c1, BaseConfig c2)
		{
			return static_cast<BaseConfig>(uint32_t(c1) | uint32_t(c2));
		}

		enum class Div
		{
			_2, _4, _8, _16, _32, _64, _128, _256
		};
		enum class Event
		{
			TxEmpty = SPI_SR_TXE,
			RxNotEmpty = SPI_SR_RXNE,
			CrcErr = SPI_SR_CRCERR,
			ModeFaultErr = SPI_SR_MODF,
			OverrunErr = SPI_SR_OVR,
			Busy = SPI_SR_BSY
		};
		enum class IRQ
		{
			TxEmpty = SPI_CR2_TXEIE,
			RxNotEmpty = SPI_CR2_RXNEIE,
			Error = SPI_CR2_ERRIE,
		};
		constexpr IRQ operator|(IRQ i1, IRQ i2)
		{
			return static_cast<IRQ>(uint32_t(i1) | uint32_t(i2));
		}
		enum class Bits
		{	_4 = 0x03,
			_5, _6, _7, _8, _9, _10,
			_11, _12, _13, _14, _15, _16
		};
		constexpr Bits operator"" _bit(unsigned long long int b)
		{
			return static_cast<Bits>(b - 1);
		}
//------=== Used for Spi class specialization ===-----

	// Remap options
		template<uint32_t BaseAddr>
		struct DefaultRemap;
		template<>
		struct DefaultRemap<SPI1_BASE>
		{
			using SckPin = Pa5;
			using MisoPin = Pa6;
			using MosiPin = Pa7;
		};

		template<uint32_t>	//interface conventions
		struct Default_Spi1_Pa5Pa6Pa7
		{
			using SckPin = Pa5;
			using MisoPin = Pa6;
			using MosiPin = Pa7;
		};
		template<uint32_t>
		struct Remap1_Spi1_Pb3Pb4Pb5
		{
			using SckPin = Pb3;
			using MisoPin = Pb4;
			using MosiPin = Pb5;
		};
#ifdef STM32F031
		template<uint32_t>
		struct Remap2_Spi1_Pb13Pb14Pb15
		{
			using SckPin = Pb13;
			using MisoPin = Pb14;
			using MosiPin = Pb15;
		};
#endif
#ifdef STM32F051
		template<>
		struct DefaultRemap<SPI2_BASE>
		{
			using SckPin = Pb13;
			using MisoPin = Pb14;
			using MosiPin = Pb15;
		};
		template<uint32_t>
		struct Default_Spi2_Pb13Pb14Pb15
		{
			using SckPin = Pb13;
			using MisoPin = Pb14;
			using MosiPin = Pb15;
		};
#endif


	//--------------
		namespace Internal {
			template<template<uint32_t> class, template<uint32_t> class>
			struct is_same_T : std::false_type { };
			template<template<uint32_t> class T1>
			struct is_same_T<T1, T1> : std::true_type { };

		//Default values
			struct InitBase
			{
				constexpr static bool sendOnly = false;
				constexpr static Bits framewidth = Bits::_16;
				template<uint32_t Base>
				using Remap_ = DefaultRemap<Base>;
				using DataT = uint16_t;
			};
		//--------------
			template<typename...>
			struct InitStruct;

			template<typename First, typename... Rest>
			struct InitStruct<First, Rest...>
			{
				using RestStuff = InitStruct<Rest...>;
				constexpr static bool sendOnly = (First::sendOnly != InitBase::sendOnly) ?
													First::sendOnly : RestStuff::sendOnly;
				constexpr static Bits framewidth = (First::framewidth != InitBase::framewidth) ?
													First::framewidth : RestStuff::framewidth;

//				template<uint32_t Base>
//				using Remap_First = typename First::template Remap_<Base>;
				template<uint32_t Base>
				using Remap_ = typename std::conditional<
					!std::is_same<typename First::template Remap_<Base>, InitBase::Remap_<Base>>::value,
								typename First::template Remap_<Base>,
								typename RestStuff::template Remap_<Base>>::type;
				using DataT = typename std::conditional<(uint32_t(framewidth) > uint32_t(Bits::_8)),
																	uint16_t, uint8_t>::type;
			};

			template<> struct InitStruct<> : InitBase
			{	};
		}//Internal

		template<template<uint32_t> class Remap__>
		struct Remap : Internal::InitBase
		{
			template<uint32_t Base>
			using Remap_ = Remap__<Base>;
		};

		template<Bits framewidth_>
		struct Framewidth : Internal::InitBase
		{
			constexpr static Bits framewidth = framewidth_;
		};

		struct SendOnly : Internal::InitBase
		{
			constexpr static bool sendOnly = true;
		};

//-----=== Main class ===---------

		template<uint32_t BaseAddr, typename... Config_>
		class Spi
		{
		protected:
			using DriverConfig = Internal::InitStruct<Config_...>;
			using Remap = typename DriverConfig::template Remap_<BaseAddr>;
			typedef typename Remap::SckPin SckPin;
			typedef typename Remap::MisoPin MisoPin;
			typedef typename Remap::MosiPin MosiPin;
			typedef typename SckPin::Port Port;
			constexpr static SPI_TypeDef* Regs()
			{
				return reinterpret_cast<SPI_TypeDef*>(BaseAddr);
			}
 		public:
			using DataT = typename DriverConfig::DataT;
			template<BaseConfig initConfig, Div div>
			static void Init()
 			{
				static_assert(!(DriverConfig::sendOnly && initConfig & RxOnly), "Configuration conflict: SendOnly & RxOnly");
				using namespace Gpio;
				constexpr static OutputConf OutputSpeed = (uint32_t(div) > uint32_t(Div::_8)) ?
												Gpio::OutputFast : Gpio::OutputFastest;
				if(BaseAddr == SPI1_BASE) RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#ifdef STM32F051
				if(BaseAddr == SPI2_BASE) RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
#endif
				Regs()->CR1 = (uint16_t)initConfig | uint16_t(div) << 3UL | (DriverConfig::sendOnly ? BidiMode | BidiOE : 0);
				if((uint16_t)(initConfig >> 16UL) || uint16_t(DriverConfig::framewidth) != uint16_t(Bits::_8))
				{
					Regs()->CR2 = (uint16_t)(initConfig >> 16UL) | uint16_t(DriverConfig::framewidth) << 8UL;
				}
				if(initConfig & MasterCfg)
				{
					if(initConfig & BidiMode || DriverConfig::sendOnly)
						Port::template SetConfig<SckPin::mask | MosiPin::mask, OutputSpeed, AltPushPull>();
					else if(initConfig & RxOnly)
						Port::template SetConfig<SckPin::mask | MisoPin::mask, OutputSpeed, AltPushPull>();
					else
						Port::template SetConfig<SckPin::mask | MisoPin::mask | MosiPin::mask, OutputSpeed, AltPushPull>();
				}
				else	//Slave
				{
					if(initConfig & BidiMode || DriverConfig::sendOnly)
						Port::template SetConfig<SckPin::mask | MisoPin::mask, OutputSpeed, AltPushPull>();
					else if(initConfig & RxOnly)
						Port::template SetConfig<SckPin::mask | MosiPin::mask, OutputSpeed, AltPushPull>();
					else
						Port::template SetConfig<SckPin::mask | MisoPin::mask | MosiPin::mask, OutputSpeed, AltPushPull>();
				}
				Enable();
 			}
			static void Enable()
			{
				Regs()->CR1 |= SPI_CR1_SPE;
			}
			static void Disable()		//Not correct, need to complete transaction
			{
				Regs()->CR1 &= ~SPI_CR1_SPE;
			}
		
			static void Send(const DataT data)
			{
				while(!Is(Event::TxEmpty))
				{  }
				Regs()->DR = data;
			}
			static DataT Receive()
			{
				static_assert(!DriverConfig::sendOnly, "Driver in SendOnly configuration");
				while(!Is(Event::RxNotEmpty))
				{	}
				return Regs()->DR;
			}
			static DataT SendReceive(const DataT data)
			{
				static_assert(!DriverConfig::sendOnly, "Driver in SendOnly configuration");
				while(!Is(Event::TxEmpty))
				{	}
				Regs()->DR = data;
				while(!Is(Event::RxNotEmpty))
				{	}
				return Regs()->DR;
			}

			static void Send(const DataT* buf, size_t size)
			{
				while(size--)
					Send(*buf++);
			}
			static void SendReceive(const DataT* txbuf, DataT* rxbuf, size_t size)
			{
				static_assert(!DriverConfig::sendOnly, "Driver in SendOnly configuration");
				Send(*txbuf);
				while(--size)
				{
					Send(*++txbuf);
					while(!Is(Event::RxNotEmpty))
					{	}
					*rxbuf++ = Regs()->DR;
				}
				*rxbuf = Receive();
			}
			static bool Is(Event event)
			{
				return Regs()->SR & uint16_t(event);
			}
			static void ClearCrcErr()
			{
				Regs()->SR &= ~SPI_SR_CRCERR;
			}
			static void Enable(IRQ mask)
			{
				Regs()->CR2 |= (uint16_t)mask;
			}
			static void Disable(IRQ mask)
			{
				Regs()->CR2 &= ~(uint16_t)mask;
			}
			static bool Busy()
			{
				return !(Regs()->SR & SPI_SR_TXE);
			}
			static bool Complete()
			{
				return !(Regs()->SR & SPI_SR_BSY);
			}
			static void SetBidiTx()
			{
				Regs()->CR1 |= BidiOE;
			}
			static void SetBidiRx()
			{
				Regs()->CR1 &= ~BidiOE;
			}
		};

//------=== Conditional members ===--------
		namespace Internal
		{
			template<typename, bool>
			struct RxMembersHelper
			{
				constexpr static bool pRx = false;	//Dummy
			};
			template<typename DataT>
			struct RxMembersHelper<DataT, false>
			{
				static volatile DataT* pRx;
			};
			template<typename DataT>
			volatile DataT* RxMembersHelper<DataT, false>::pRx;

			template<typename... Config>
			struct RxMembers : RxMembersHelper<typename InitStruct<Config...>::DataT,
															InitStruct<Config...>::sendOnly>
			{
//				using RxMembersHelper<typename InitStruct<Config...>::DataT,
//													InitStruct<Config...>::sendOnly>::pRx;
			};
		}

//------------------------------------------

		template<uint32_t BaseAddr, typename... Config_>
		class SpiIrq : private Internal::RxMembers<Config_...>, public Spi<BaseAddr, Config_...>
		{
		public:
			using DataT = typename Spi<BaseAddr, Config_...>::DataT;
		protected:
			using Base = Spi<BaseAddr, Config_...>;
			using DriverConfig = typename Base::DriverConfig;
			using Base::Regs;
			static const volatile DataT* pTx;
			using Internal::RxMembers<Config_...>::pRx;
			static volatile uint32_t datacount;
			static void RxHandler(const bool)
			{	}
			static void RxHandler(volatile DataT* prx)
			{
				if(Is(Event::RxNotEmpty))
				{
					*prx++ = Regs()->DR;
				}
			}
		public:
			using Base::Enable;	using Base::Disable; using Base::Is; using Base::Send;
			template<BaseConfig config, Div div>
			static void Init()
			{
				Base::template Init<config, div>();
				enum{ irq = SPI1_BASE ? SPI1_IRQn :
  #ifdef STM32F051
							SPI2_BASE ? SPI2_IRQn :
#endif
												0 };
				STATIC_ASSERT(irq != 0);
				NVIC_EnableIRQ((IRQn)irq);
			}
			static bool Send(const DataT* buf, size_t size)
			{
				if(datacount) return false;
				Regs()->DR = *buf;
				datacount = size - 1;
				pTx = buf;
				if(!DriverConfig::sendOnly)
					Disable(IRQ::RxNotEmpty);
				Enable(IRQ::TxEmpty);
				return true;
			}
			static bool SendReceive(const DataT* txbuf, DataT* rxbuf, size_t size)
			{
				static_assert(!DriverConfig::sendOnly, "SendReceive is not work in SendOnly config");
				if(datacount) return false;
				pRx = rxbuf;
				pTx = txbuf;
				datacount = size - 1;
				Regs()->DR = *txbuf;
				Enable(IRQ::TxEmpty | IRQ::RxNotEmpty);
				return true;
			}
			static void Irq()
			{
				if(Is(Event::TxEmpty))
 				{
					if(datacount)
					{
						Regs()->DR = *++pTx;
						--datacount;
					}
					else
						Disable(IRQ::TxEmpty);
 				}
				RxHandler(pRx);
			}
		};
		template<uint32_t BaseAddr, typename... Config_>
		volatile uint32_t SpiIrq<BaseAddr, Config_...>::datacount;

		template<uint32_t BaseAddr, typename... Config_>
		const volatile typename SpiIrq<BaseAddr, Config_...>::DataT*
			SpiIrq<BaseAddr, Config_...>::pTx;

/*
		template<typename spiconf>
		class SpiDma : public Spi<spiconf>
		{
		protected:
			typedef Spi<spiconf> SBase;
			typedef typename Private::DmaTraits<spiconf::BaseAddr>::DmaRx DmaRx;
			typedef typename Private::DmaTraits<spiconf::BaseAddr>::DmaTx DmaTx;
			using SBase::Regs;
			USINGBASEFUNC(SBase, bool, IsEvent, Events)
		public:
			typedef typename SBase::DataT DataT;
			USINGBASEFUNC_NORET(SBase, EnableIRQ, IRQs)
			USINGBASEFUNC_NORET(SBase, DisableIRQ, IRQs)
			template<Cfg config, BaudDivider div = Div2>
			static void Init()
			{
				SBase:: template Init<config, div>();
				Regs()->CR2 = ((uint32_t)DmaTxEnable | (spiconf::SendOnly ? 0 : DmaRxEnable)) >> 16UL;
				{
					using namespace Dmas;
					DmaTx::template Init<Dmas::Cfg(MemInc | FromMem),
						spiconf::Width ? Mem16bits : Mem8bits,
						spiconf::Width ? Per16bits : Per8bits>();
					if(!spiconf::SendOnly)
						DmaRx::template Init<Dmas::Cfg(MemInc | FromPeriph),
						spiconf::Width ? Mem16bits : Mem8bits,
						spiconf::Width ? Per16bits : Per8bits>();
					DmaTx::SetPeriphAddr(&Regs()->DR);
				}
			}
			static bool Send(const DataT* buf, uint32_t size)
			{
				if(DmaTx::Busy()) return false;
				DmaTx::Disable();
				DmaTx::SetMemAddr(buf);
				DmaTx::SetCounter(size);
				DmaTx::Enable();
				return true;
			}
			static bool SendReceive(const DataT* txbuf, DataT* rxbuf, uint32_t size)
			{
				if(DmaTx::Busy()) return false;
				DmaTx::Disable();
				DmaTx::SetPerMemAddr(&Regs()->DR, txbuf);
				DmaTx::SetCounter(size);
				DmaRx::Disable();
				DmaRx::SetPerMemAddr(&Regs()->DR, rxbuf);
				DmaRx::SetCounter(size);
				DmaTx::Enable();
				DmaRx::Enable();
				return true;
			}
			static void MemIncrementDisable()
			{
				DmaTx::MemIncrementDisable();
			}
			static void MemIncrementEnable()
			{
				DmaTx::MemIncrementEnable();
			}
			static uint16_t TxBusy()
			{
				return DmaTx::Busy();
			}
		};
*/
	}//Spis

}//Mcucpp
