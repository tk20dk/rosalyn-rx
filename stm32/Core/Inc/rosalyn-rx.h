#ifndef ROSALYN_RX_H__
#define ROSALYN_RX_H__

#include "nvdata.h"
#include "system.h"
#include "sx1268.h"


extern "C" SPI_HandleTypeDef hspi1;

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
  void HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin );

private:
  bool Failsafe;
  bool RadioFlag;
  TSx1268 Radio;
  TNvData NvData;
};

#endif // ROSALYN_RX_H__
