#ifndef ROSALYN_RX_H__
#define ROSALYN_RX_H__

#include "sbus.h"
#include "nvdata.h"
#include "system.h"
#include "sx1268.h"
#include "aes-crypto.h"


class TRosalynRx
{
  static uint32_t const MinPPM = 900;
  static uint32_t const MaxPPM = 2100;
  static uint32_t const NoOfPPMs = 8;

public:
  TRosalynRx();

  void Loop();
  void Setup();
  void HmiLoop();
  void HmiError( uint32_t const Interval = 0 );
  void HmiStatus( uint32_t const Interval = 0 );
  void RadioEvent( TRadioEvent const Event );

  void SysTick_Handler();
  void EXTI2_3_IRQHandler();
  void EXTI4_15_IRQHandler();
  void USART2_IRQHandler();
  void USART3_4_IRQHandler();

private:
  TNvData NvData;
  bool IcmFlag;;
  bool TimerFlag;
  bool RadioFlag;
  bool SerialFlag;
  TSx1268 Radio;
  uint32_t TimeoutHmiError;
  uint32_t TimeoutHmiStatus;
  TSbusData SbusDataUpstream;
  TSbusData SbusDataDownstream;
  TAesCrypto AesCrypto;
  TSbusSerial SbusSerial;
};

#endif // ROSALYN_RX_H__
