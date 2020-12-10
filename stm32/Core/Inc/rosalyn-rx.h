#ifndef ROSALYN_RX_H__
#define ROSALYN_RX_H__

#include "nvdata.h"
#include "system.h"
#include "sx1268.h"


class TRosalynRx
{
  static uint32_t const MinPPM = 900;
  static uint32_t const MaxPPM = 2100;
  static uint32_t const NoOfPPMs = 8;

public:
  TRosalynRx();

  void Loop();
  void Setup();
  void RadioEvent( TRadioEvent const Event );

  void SysTick_Handler();
  void EXTI2_3_IRQHandler();
  void EXTI4_15_IRQHandler();
  void USART2_IRQHandler();
  void USART3_4_IRQHandler();

private:
  bool IcmFlag;;
  bool RadioFlag;
  TSx1268 Radio;
  TNvData NvData;
};

#endif // ROSALYN_RX_H__
