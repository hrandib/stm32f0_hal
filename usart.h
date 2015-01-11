// STM32F030 & STM32F031 & STM32F051
#pragma once

#include "stm32f0xx.h"
#include "utils.h"
#include "streams.h"
#include "gpio.h"
#include "circularBuffer.h"

namespace Mcucpp
{
namespace Usarts
 {
	using std::is_same;
	enum BaseConfig : uint64_t
	{
		DataBits8 = 0,
		DataBits9 = USART_CR1_M,

		ParityNo = 0,
		ParityEven = USART_CR1_PCE,
		ParityOdd = USART_CR1_PCE | USART_CR1_PS,

		OneStopBit = 0,
		OneAndHalfStopBits = USART_CR2_STOP_0 | USART_CR2_STOP_1 << 16UL,		//1.5 Stop bits are not available for all UARTs.
		TwoStopBits	= USART_CR2_STOP_1 << 16UL,

		TxEnable = USART_CR1_TE,
		RxEnable = USART_CR1_RE,
		RxTxEnable  = RxEnable | TxEnable,

		SingleWire = (uint64_t)USART_CR3_HDSEL << 32UL,

		//Enable Tx, Rx, 8N1
		DefaultCfg = RxTxEnable | DataBits8 | OneStopBit,

		Enable = USART_CR1_UE
	};

	constexpr BaseConfig operator|(BaseConfig c1, BaseConfig c2)
	{
		return static_cast<BaseConfig>(static_cast<uint32_t>(c1) | c2);
	}

	template<uint32_t baud = 9600UL>
	struct BaudRate
	{
		enum { Div = F_CPU / baud };
	private:
		static_assert(Div <= __UINT16_MAX__ && Div > 0x0F, "Baudrate divider must be in range 16...65535");
	};

	enum class OldCfg : uint64_t	//TODO: Move this functionality to Usart::Init template features
	{

//		---=== USART CR1 ===---			
		EnabledInStopMode = USART_CR1_UESM,

		WakeIdleLine = 0,
		WakeAddressMark = USART_CR1_WAKE,


		MuteMode = USART_CR1_MME,
		Over8 = USART_CR1_OVER8,

//		---=== UART CR2 ===---			
		LinEnable = USART_CR2_LINEN << 16UL,

	// Should not be written while the transmitter is enabled
		ClkEnable = (uint32_t)USART_CR2_CLKEN << 16UL,
		CPOL_0_Idle = 0,
		CPOL_1_Idle = (uint32_t)USART_CR2_CPOL << 16UL,
		CPHA_0 = 0,								// The first clock transition is the first data capture edge
		CPHA_1 = (uint32_t)USART_CR2_CPHA << 16UL,	// The second clock transition is the first data capture edge
		LastBitPulseOn = (uint32_t)USART_CR2_LBCL << 16UL,
		
		LinBreak10bit = 0,
		LinBreak11bit = (uint32_t)USART_CR2_LBDL << 16UL,

//		---=== UART CR3 ===---			
		CtsEnable = (uint64_t)USART_CR3_CTSE << 32UL,
		RtsEnable = (uint64_t)USART_CR3_RTSE << 32UL,

		DmaTxEnable = (uint64_t)USART_CR3_DMAT << 32UL,
		DmaRxEnable = (uint64_t)USART_CR3_DMAR << 32UL,
		
// 		SCModeEnable
// 		SCNackEnable


		//IrDALowPower
		//IrDaEnable

	};

	enum class Event
	{
		RxEnableAck = USART_ISR_REACK,
		TxEnableAck = USART_ISR_TEACK,
		WakeupFromStop = USART_ISR_WUF,
		RxWakeupFromMute = USART_ISR_RWU,
		SendBreak = USART_ISR_SBKF,
		CharMatch = USART_ISR_CMF,
		Busy = USART_ISR_BUSY,
		AutoBaud = USART_ISR_ABRF,
		AutoBaudErr = USART_ISR_ABRE,
		EndOfBlock = USART_ISR_EOBF,
		RxTimeout = USART_ISR_RTOF,
		Cts = USART_ISR_CTS,
		CtsIF = USART_ISR_CTSIF,
		LinBreak = USART_ISR_LBD,
		TxEmpty = USART_ISR_TXE,
		TxComplete = USART_ISR_TC,
		RxNotEmpty = USART_ISR_RXNE,
		Idle = USART_ISR_IDLE,
		OverrunErr = USART_ISR_ORE,
		NoiseErr = USART_ISR_NE,
		FrameErr = USART_ISR_FE,
		ParityErr = USART_ISR_PE
	};
	constexpr Event operator|(Event ev1, Event ev2)
	{
		return (Event)(uint32_t(ev1) | uint32_t(ev2));
	}

	enum class IRQ : uint64_t
	{
		//		---=== UART CR1 ===---
		EndOfBlock = USART_CR1_EOBIE,
		RxTimout = USART_CR1_RTOIE,
		CharMatch = USART_CR1_CMIE,
		Parity = USART_CR1_PEIE,
		TxEmpty = USART_CR1_TXEIE,
		TxComplete = USART_CR1_TCIE,
		RxNotEmpty = USART_CR1_RXNEIE,
		Idle = USART_CR1_IDLEIE,
		//		---=== UART CR2 ===---
		LinBreak = USART_CR2_LBDIE >> 6UL, // = 0x01
		//		---=== UART CR3 ===---
		WakeupFromStop = (uint64_t)USART_CR3_WUFIE << 32UL,
		Cts = (uint64_t)USART_CR3_CTSIE << 32UL,
		Errors = (uint64_t)USART_CR3_EIE << 32UL,
		None = 0
	};
	constexpr IRQ operator|(IRQ irq1, IRQ irq2)
	{
		return (IRQ)(uint32_t(irq1) | uint32_t(irq2));
	}

	enum class Request
	{
		TxDataFlush = USART_RQR_TXFRQ,
		RxDataFlush = USART_RQR_RXFRQ,
		MuteMode = USART_RQR_MMRQ,
		SendBreak = USART_RQR_SBKRQ,
		AutoBaudRate = USART_RQR_ABRRQ
	};

// Using for Usart class specialization

	template<typename TxPin_, typename RxPin_>
	struct Remap
	{
		using TxPin = TxPin_;
		using RxPin = RxPin_;
	};

	template<uint32_t baseaddr>
	struct DefaultRemap;
	template<> struct DefaultRemap<USART1_BASE>
	{
		using TxPin = Pb6;
		using RxPin = Pb7;
	};	
	struct RemapUsart1_Pa9Pa10
	{
		using TxPin = Pa9;
		using RxPin = Pa10;
	};
#if defined STM32F031 || defined STM32F030		//valid for STM32F030x4 STM32F030x6
	struct RemapUsart1_Pa2Pa3
	{
		using TxPin = Pa2;
		using RxPin = Pa3;
	};
	struct RemapUsart1_Pa14Pa15
	{
		using TxPin = Pa14;
		using RxPin = Pa15;
	};
#endif
#if defined STM32F051 || defined STM32F030		//valid for STM32F030x8
	template<> struct DefaultRemap<USART2_BASE>
	{
		using TxPin = Pa2;
		using RxPin = Pa3;
	};
	struct RemapUsart2_Pa2Pa3
	{
		using TxPin = Pa2;
		using RxPin = Pa3;
	};
	struct RemapUsart2_Pa14Pa15
	{
		using TxPin = Pa14;
		using RxPin = Pa15;
	};
#endif
//----------------------

// Not necessary init options
	namespace Private {
		template<typename...>
		struct UsartInitStruct;
		template<typename First, typename... Rest>
		struct UsartInitStruct<First, Rest...>
		{
			using RestStuff = UsartInitStruct<Rest...>;
			constexpr static uint32_t CR1 = First::CR1 | RestStuff::CR1;
			constexpr static uint32_t CR2 = First::CR2 | RestStuff::CR2;
			constexpr static uint32_t CR3 = First::CR3 | RestStuff::CR3;
			template<typename Usart>
			struct Wrapper
			{
				static void Apply()
				{
					First::template Wrapper<Usart>::Apply();
					RestStuff::template Wrapper<Usart>::Apply();
				}
			};
		};
		template<> struct UsartInitStruct<>
		{
			constexpr static uint32_t CR1 = 0;
			constexpr static uint32_t CR2 = 0;
			constexpr static uint32_t CR3 = 0;
			template<typename> struct Wrapper
			{
				static void Apply() { }
			};
		};
		using InitStructBase = UsartInitStruct<>;
	}

	enum class Polarity
	{
		Data = USART_CR2_DATAINV,
		Tx = USART_CR2_TXINV,
		Rx = USART_CR2_RXINV
	};
	constexpr Polarity operator|(Polarity p1, Polarity p2)
	{
		return static_cast<Polarity>(static_cast<uint32_t>(p1) | static_cast<uint32_t>(p2));
	}
	template<Polarity pol>
	struct InvertPolarity : public Private::InitStructBase
	{
		constexpr static uint32_t CR2 = static_cast<uint32_t>(pol);
	};

	template<uint32_t timeout>
	struct EnableRxTimeoutIRQ : public Private::InitStructBase
	{
		constexpr static uint32_t CR1 = USART_CR1_RTOIE;
		template<typename Usart>
		struct Wrapper : public Usart
		{
			static void Apply()
			{
				Usart::Regs()->RTOR = timeout;
			}
		};
	};

	template<typename Pin, uint32_t Prolong = 0, bool ActiveLow = false>
	struct DriverEnable : public Private::InitStructBase
	{
		static_assert(is_same<Pin, Pa1>::value || is_same<Pin, Pa12>::value, "Driver Enable pin wrong");
		static_assert(0 <= Prolong && Prolong <= 31, "Prolong of DE signal is not in range. Range: 0...31");
		constexpr static uint32_t CR1 = Prolong << 21UL | Prolong << 16UL;
		constexpr static uint32_t CR3 = USART_CR3_DEM | (ActiveLow ? USART_CR3_DEP : 0);
		template<typename>
		struct Wrapper
		{
			static void Apply()
			{
				Pin::template SetConfig<Gpio::OutputFast, Gpio::AltPushPull>();
				Pin::template AltFuncNumber<Gpio::AF::_1>();
			}
		};
	};

	enum class Dma
	{
		Tx = USART_CR3_DMAT,
		Rx = USART_CR3_DMAR
	};
	constexpr Dma operator|(Dma p1, Dma p2)
	{
		return static_cast<Dma>(static_cast<uint32_t>(p1) | static_cast<uint32_t>(p2));
	}
	template<Dma dma>
	struct EnableDma : public Private::InitStructBase
	{
		constexpr static uint32_t CR3 = uint32_t(dma);
//----=== Access to protected Usart members example ===-----------
/*
		template<typename Usart>
		struct Wrapper : public Usart
		{
			static void Apply()
			{
				__NOP();
				Usart::Regs()->CR2 = Usart::BaseAddress;
				__NOP();
			}
		};
*/	};

//---------------------

// Class with polling routines
	template<uint32_t BaseAddr, typename Remap_ = DefaultRemap<BaseAddr>>
	class Usart
	{
	protected:
		using Self = Usart<BaseAddr, Remap_>;
		using TxPin = typename Remap_::TxPin;
		using RxPin = typename Remap_::RxPin;
		constexpr static USART_TypeDef* Regs()
		{
			return reinterpret_cast<USART_TypeDef*>(BaseAddr);
		}
		constexpr static uint32_t BaseAddress = BaseAddr;
	public:
		template<BaseConfig conf, typename BaudRate_ = BaudRate<9600UL>, typename... Config>
		static void Init()
		{
			using namespace Gpio;
			using Port = typename TxPin::Port; //Common for Tx, Rx
			using InitStruct = Private::UsartInitStruct<Config...>;
		// Enable Peripheral Clock
			if (BaseAddr == USART1_BASE) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			else RCC->APB1ENR |= 1 << (((BaseAddr >> 8) & 0xFF) / 4);			//Usarts 2 - 4 bitmask
		// Set Baud Rate
			Regs()->BRR = BaudRate_::Div;
		// Main Init
			constexpr static uint32_t CR2 = InitStruct::CR2 | (uint16_t)(conf >> 16UL);
			if(CR2)	Regs()->CR2 = CR2;
			constexpr static uint32_t CR3 = InitStruct::CR3 | conf >> 32UL;
			if(CR3)	Regs()->CR3 = CR3;
			Regs()->CR1 = (uint32_t)conf | InitStruct::CR1;
		// Custom init steps
			InitStruct::template Wrapper<Self>::Apply();
		// Gpio Init
			if(BaseAddr == USART1_BASE)
			{
				static const bool isRemap = (is_same<TxPin, Pa2>::value & is_same<RxPin, Pa3>::value) ||
											(is_same<TxPin, Pa9>::value & is_same<RxPin, Pa10>::value) ||
											(is_same<TxPin, Pa14>::value & is_same<RxPin, Pa15>::value);
				static const bool noRemap = is_same<TxPin, Pb6>::value & is_same<RxPin, Pb7>::value;
				static_assert(isRemap | noRemap, "Wrong Remap");
				if(isRemap)
				{
					Port::template AltFuncNumber<TxPin::mask | RxPin::mask, AF::_1>();
				}
			}
			else if(BaseAddr == USART2_BASE)
			{
				//TODO: USART2 support not implemented yet.
			}
			if (!(conf & SingleWire)) Port::template SetConfig<TxPin::mask | RxPin::mask, OutputFast, AltPushPull>();
			else TxPin::template SetConfig<OutputFast, AltPushPull>();
			Enable();
		}

		template<uint8_t addr>
		static void SetNodeAddress()	// Incompatible With LIN mode
		{
			Regs()->CR2 = (Regs()->CR2 & 0x00FFFFF) | addr << 24UL;
		}

		template<Event event>
		static bool IsEvent()
		{
			return Regs()->ISR & (uint32_t)event;
		}
			
		template<Event event>
		static void ClearEvent()
		{
			Regs()->ICR = static_cast<uint32_t>(event);
		}

		template<Request req>
		static void SendRequest()
		{
			Regs()->RQR = uint32_t(req);
		}

		template <IRQ mask>
		static void EnableIRQ()
		{
			if((uint32_t)mask & 0xFFFFFFFE) Regs()->CR1 |= (uint32_t)mask;
			if((uint32_t)mask & 0x01) Regs()->CR2 |=  USART_CR2_LBDIE;
			if((uint64_t)mask >> 32UL) Regs()->CR3 |= (uint64_t)mask >> 32UL;
		}
		template <IRQ mask>
		static void DisableIRQ()
		{
			if((uint32_t)mask & 0xFFFFFFFE) Regs()->CR1 &= ~((uint32_t)mask);
			if((uint32_t)mask & 0x01) Regs()->CR2 &=  ~USART_CR2_LBDIE;
			if((uint64_t)mask >> 32UL) Regs()->CR3 &= ~((uint64_t)mask >> 32UL);
		}
		static void Enable()
		{
			Regs()->CR1 |= BaseConfig::Enable;
		}
		static void Disable()
		{
			Regs()->CR1 &= ~BaseConfig::Enable;
		}
		// Functions with polling
		static void Putch(uint8_t ch)
		{
			while(!(IsEvent<Event::TxEmpty>()));
			Regs()->TDR = ch;
		}
		static void Puts(const char *s)
		{
			while(*s)
			{
				Putch(*s++);
			}
		}
		static void Putbuf(const uint8_t *buf, uint16_t size)
		{
			while(size--)
			{
				Putch(*buf++);
			}
		}
		static uint8_t Getch()
		{
			while (!(IsEvent<Event::RxNotEmpty>()));
#ifdef USARTECHO
			Regs()->TDR = Regs()->RDR;
#endif
			return Regs()->RDR;
 		}
	};

//---------------------

// Interrupts and cyclic buffer used
	template< uint32_t BaseAddr,
			 typename Remap_ = DefaultRemap<BaseAddr>,
			 uint32_t TxBufSize = 16,
			 uint32_t RxBufSize = TxBufSize >
	class UsartIrq : public Usart<BaseAddr, Remap_>
	{
	public:
		enum
		{ 
			TXBUFSIZE = TxBufSize,
			RXBUFSIZE = RxBufSize
		};
	protected:
		typedef Usart<BaseAddr, Remap_> UBase;
		using UBase::Regs;
		USINGBASEFUNC(UBase, bool, IsEvent, Event)
		USINGBASEFUNC_NORET(UBase, EnableIRQ, IRQ)
		USINGBASEFUNC_NORET(UBase, DisableIRQ, IRQ)
		static CircularBuffer<TxBufSize> txbuf;
		static CircularBuffer<RxBufSize> rxbuf;
	public:
		template<BaseConfig conf, typename BaudRate_ = BaudRate<9600UL>, typename... Config>
		static void Init()
		{
			UBase::template Init<conf, BaudRate_, Config...>();
			enum{ irq =
				(BaseAddr == USART1_BASE) ? USART1_IRQn :
#ifdef STM32F051
				(BaseAddr == USART2_BASE) ? USART2_IRQn :
#endif
															0 };
			STATIC_ASSERT(irq != 0);
			NVIC_EnableIRQ((IRQn)irq);
			NVIC_SetPriority((IRQn)irq, 1);
			EnableIRQ<IRQ::RxNotEmpty>();
		}

		static bool Putch(const uint8_t c)
		{
			bool st = txbuf.Write(c);
			EnableIRQ<IRQ::TxEmpty>();
			return st;
		}

		static bool Puts(const uint8_t* s)
		{
			while(*s)
			{
				if(!txbuf.Write(*s++)) return false;
			}
			EnableIRQ<IRQ::TxEmpty>();
			return true;
		}
		static bool Puts(const char* s)
		{
			return Puts((const uint8_t*)s);
		}
		template<DataFormat system>
		static bool Puts(int32_t value)
		{
			using namespace PrivateUtils;
			uint8_t buf[UtoaTraits<system>::asize + 1] = { 0 };
			uint8_t* ptr = IO::itoa(value, buf, UtoaTraits<system>::base);
			while(*ptr)
			{
				if(!txbuf.Write(*ptr++)) return false;
			}
			EnableIRQ<IRQ::TxEmpty>();
			return true;
		}
		
		static bool Putbuf(const uint8_t *buf, typename CircularBuffer<TxBufSize>::INDEX_T size)
		{
			while(size--)
			{
				if(!txbuf.Write(*buf++)) return false;
			}
			EnableIRQ<IRQ::TxEmpty>();
			return true;
		}		
		template<typename T>
		static bool Putbuf(const T* buf, typename CircularBuffer<TxBufSize>::INDEX_T size)
		{
			STATIC_ASSERT(sizeof(T) == 1);
			return Putbuf((const uint8_t*)buf, size);
 		}

		static bool NewLine()
		{
			if(!(Putch('\r') && Putch('\n'))) return false;
			return true;//Puts("\n\r");
		}

		static bool Getch(uint8_t &c)
		{
			return rxbuf.Read(c);
		}
		static uint8_t BytesReceived()
		{
			return rxbuf.Count();
		}
		static void FlushRxBuf()
		{
			rxbuf.Clear();
		}
		static void Irq()
		{
			if(IsEvent<Event::TxEmpty>())
			{
				uint8_t c;
				if(txbuf.Read(c))
					Regs()->TDR = c;
				else
					DisableIRQ<IRQ::TxEmpty>();
			}
			if(IsEvent<Event::RxNotEmpty>())
			{
				uint8_t ch = Regs()->RDR;
				if(!rxbuf.Write(ch)) //buffer overlow
				{
					//TODO: error handling
					rxbuf.Clear();
				}
#ifdef USARTECHO
				Regs()->TDR = ch;
#endif
			}
		}
	};

	template< uint32_t BaseAddr, typename Remap_, uint32_t TxBufSize, uint32_t RxBufSize>
	CircularBuffer<TxBufSize> UsartIrq<BaseAddr, Remap_, TxBufSize, RxBufSize>::txbuf;
	template< uint32_t BaseAddr, typename Remap_, uint32_t TxBufSize, uint32_t RxBufSize>
	CircularBuffer<RxBufSize> UsartIrq<BaseAddr, Remap_, TxBufSize, RxBufSize>::rxbuf;

//--------------------------
	//TODO: implement DMA handling

/* 	template<uint32_t BaseAddr, RemapCfg rem = NoRemap, typename DEpin = Nullpin, uint32_t TxBufSize = 16, uint32_t RxBufSize = TxBufSize>
	class UsartDma : public Usart < BaseAddr, rem, DEpin >
	{
	private:
		typedef Usart<BaseAddr, rem, DEpin> UBase;
		using UBase::Regs;
		USINGBASEFUNC(UBase, bool, IsEvent, Events)
			USINGBASEFUNC_NORET(UBase, EnableIRQ, IRQs)
			USINGBASEFUNC_NORET(UBase, DisableIRQ, IRQs)
			static CircularBuffer<TxBufSize> txbuf;
		static CircularBuffer<RxBufSize> rxbuf;
	public:
		template<Cfg config, BaudRate baud = 9600UL>
			static void Init()
		{
			UBase::template Init<config, baud>();
			enum{
				irq =
				(BaseAddr == USART1_BASE) ? USART1_IRQn :
				(BaseAddr == USART2_BASE) ? USART2_IRQn :
				(BaseAddr == USART3_BASE) ? USART3_IRQn :
 #if defined STM32F10X_CL || defined STM32F10X_XL || defined STM32F10X_HD_VL || defined STM32F10X_HD
				(BaseAddr == UART4_BASE) ? UART4_IRQn :
				(BaseAddr == UART5_BASE) ? UART5_IRQn :
 #endif
				0
			};
			NVIC_EnableIRQ((IRQn)irq);
			//			UBase::template EnableIRQ<(IRQs)(RxNEIRQ | TxEmptyIRQ)>();
		}

		static bool Putch(uint8_t c)
		{
			if(txbuf.IsEmpty())
			{
				UBase::Putch(c);
				EnableIRQ<TxEmptyIRQ>();
				return true;
			}
			else
				return txbuf.Write(c);
		}
		static void Puts(const uint8_t *s)
		{
			while(*s)
			{
				Putch(*s++);
			}
		}
		static void Putbuf(const uint8_t *buf, typename CircularBuffer<TxBufSize>::INDEX_T size)
		{
			while(size--)
			{
				Putch(*buf++);
			}
		}
		template<typename T>
			static void Putbuf(const T* buf, typename CircularBuffer<TxBufSize>::INDEX_T size)
		{
			STATIC_ASSERT(sizeof(T) == 1);
			Putbuf((const uint8_t*)buf, size);
		}

		static bool Getch(uint8_t &c)
		{
			return rxbuf.Read(c);
		}

		static uint8_t BytesReceived()
		{
			return rxbuf.Count();
		}

		inline static void Irq()
		{
			if(IsEvent<TxEmptyEv>())
			{
				uint8_t c;
				if(txbuf.Read(c))
					Regs()->DR = c;
				else
					DisableIRQ<TxEmptyIRQ>();
			}
			if(IsEvent<RxNEEv>())
			{
				if(!rxbuf.Write(Regs()->DR)) //buffer overlow
				{
					TODO_: error handling
					rxbuf.Clear();
				}
			}
		}
	};
 */

	//return null terminated string(builded from line) from stream if received, nullptr if not.

// Helper utils
	template<typename Usart, uint32_t BUFSIZE = Usart::RXBUFSIZE>
	inline uint8_t* GetStr()
	{
		static uint8_t buf[BUFSIZE];
		static typename SelectSizeForLength<BUFSIZE>::type i = 0;
		uint8_t ch;
		if(Usart::Getch(ch))
		{
			if(i != (BUFSIZE - 1))
			{
				if(ch == '\r') return nullptr;
				if(ch == '\n') ch = '\0';
				buf[i++] = ch;
				if(ch == '\0')
				{
					i = 0;
					return buf;
				}
			}
			else
			{
				i = 0;
				return nullptr;
			}
		}
		return nullptr;
	}

//--------------------------------

	}//Usarts
}//Mcucpp

