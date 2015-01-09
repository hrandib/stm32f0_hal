// STM32F031 & STM32F051
#pragma once
#include <stm32f0xx.h>

namespace Mcucpp
{
	namespace Dmas
	{
		enum Cfg
		{
			Mem2Mem = DMA_CCR_MEM2MEM,
			MemInc = DMA_CCR_MINC,
			PerInc = DMA_CCR_PINC,
			Circular = DMA_CCR_CIRC,
			FromPeriph = 0,
			FromMem = DMA_CCR_DIR,
			TransferErrIRQ = DMA_CCR_TEIE,
			HalfTransferIRQ = DMA_CCR_HTIE,
			TransferCompleteIRQ = DMA_CCR_TCIE
		};
		constexpr Cfg operator|(Cfg c1, Cfg c2)
		{ return static_cast<Cfg>(uint32_t(c1) | uint32_t(c2)); }

		enum Priority
		{
			Low,
			Medium,
			High,
			VeryHigh
		};
		enum MemSize
		{
			Mem8bits,
			Mem16bits,
			Mem32bits
		};
		enum PeriphSize
		{	
			Per8bits,
			Per16bits,
			Per32bits
		};

		namespace Private
		{
			template<uint32_t BaseAddr>
			class Dma
			{
			protected:
				inline static DMA_Channel_TypeDef* Regs()
				{
					return reinterpret_cast<DMA_Channel_TypeDef*>(BaseAddr);
				}
			public:
/* 				struct MemIncrementOff
				{
					inline MemIncrementOff()
					{
						MemIncrementDisable();
					}
					inline ~MemIncrementOff()
					{
						MemIncrementEnable();
					}
				};
*/
				template<Cfg cfg, MemSize msize, PeriphSize psize, Priority pr = Low>
				static void Init()
				{
					Regs()->CCR = cfg | msize << 10UL | psize << 8UL | pr << 12UL;
				}
				inline static void MemIncrementDisable()
				{
					Disable();
					Regs()->CCR &= ~MemInc;
				}
				inline static void MemIncrementEnable()
				{
					Disable();
					Regs()->CCR |= MemInc;
				}
				template<typename T1, typename T2>
				inline static void SetPerMemAddr(T1* peraddr, T2* memaddr)
				{
					Regs()->CPAR = (uint32_t)peraddr;
					Regs()->CMAR = (uint32_t)memaddr;
				}
				template<typename T>
				inline static void SetPeriphAddr(T* peraddr)
				{
					Regs()->CPAR = (uint32_t)peraddr;
				}
				template<typename T>
				inline static void SetMemAddr(T* memaddr)
				{
					Regs()->CMAR = (uint32_t)memaddr;
				}
				inline static void SetCounter(uint16_t count)
				{
					Regs()->CNDTR = count;
				}
				inline static void Enable()
				{
					Regs()->CCR |= DMA_CCR_EN;
				}
				inline static void Disable()
				{
					Regs()->CCR &= ~DMA_CCR_EN;
				}
				inline static uint16_t Busy()
				{
					return Regs()->CNDTR;
				}
			};
		}
		
		inline static void EnableClock()
		{
			RCC->AHBENR |= RCC_AHBENR_DMAEN;
		}

		typedef Private::Dma<DMA1_Channel1_BASE> DmaCh1;
		typedef Private::Dma<DMA1_Channel2_BASE> DmaCh2;
		typedef Private::Dma<DMA1_Channel3_BASE> DmaCh3;
		typedef Private::Dma<DMA1_Channel4_BASE> DmaCh4;
		typedef Private::Dma<DMA1_Channel5_BASE> DmaCh5;

	}
}
