#include "rosalyn-rx.h"


TDriverSpi Spi;
TRosalynRx RosalynRx;

void TRosalynRx::Loop()
{
  if( IcmFlag )
  {
    IcmFlag = false;
  }

  if( RadioFlag )
  {
    RadioFlag = false;
    Radio.Interrupt();
  }
}

void TRosalynRx::RadioEvent( TRadioEvent const Event )
{
  uint8_t Buffer[ 256 ];

  if( Event == TRadioEvent::RxDone )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );

    Radio.Transmit( Buffer, Length );
//    Radio.Receive();
  }

  if( Event == TRadioEvent::TxDone )
  {
    HmiStatus( true );
    LL_mDelay( 5 );
    HmiStatus( false );

    Radio.Receive();
  }

  if( Event == TRadioEvent::Timeout )
  {
  }

  if( Event == TRadioEvent::CrcError )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    HmiError( true );
    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    HmiError( true );
    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }
}

void TRosalynRx::Setup()
{
//  NvData.Load();
  UartPrintf( "RosalynTX\n" );
  HmiStatus( true );

  Spi.Setup();

  if( Radio.Setup( NvData.Modulation[ 2 ], NvData.TxPower, NvData.Channel ))
  {
//    uint8_t Buffer[128];
//    Radio.Transmit( Buffer, sizeof( Buffer ));
    Radio.Receive();
  }
}

void TRosalynRx::SysTick_Handler()
{
}

void TRosalynRx::EXTI2_3_IRQHandler()
{
  if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_2 ) != RESET )
  {
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_2 );
    IcmFlag = true;
  }
}

void TRosalynRx::EXTI4_15_IRQHandler()
{
  if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_10 ) != RESET )
  {
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_10 );
    RadioFlag = true;
  }
}

void TRosalynRx::USART2_IRQHandler()
{
}

void TRosalynRx::USART3_4_IRQHandler()
{
}

TRosalynRx::TRosalynRx() :
  IcmFlag( false ),
  RadioFlag( false ),
  Radio(
    433050000,
	RADIO_NSS_GPIO_Port,
	RADIO_NSS_Pin,
	RADIO_NRST_GPIO_Port,
	RADIO_NRST_Pin,
	RADIO_BUSY_GPIO_Port,
	RADIO_BUSY_Pin,
	RADIO_RXEN_GPIO_Port,
	RADIO_RXEN_Pin,
	RADIO_TXEN_GPIO_Port,
	RADIO_TXEN_Pin,
    std::bind( &TRosalynRx::RadioEvent, this, std::placeholders::_1 )),
  NvData()
{
}

extern "C" void RosalynRxLoop()
{
  RosalynRx.Loop();
}

extern "C" void RosalynRxSetup()
{
  RosalynRx.Setup();
}

extern "C" void SysTick_Handler()
{
  RosalynRx.SysTick_Handler();
}

extern "C" void EXTI2_3_IRQHandler()
{
  RosalynRx.EXTI2_3_IRQHandler();
}

extern "C" void EXTI4_15_IRQHandler()
{
  RosalynRx.EXTI4_15_IRQHandler();
}

extern "C" void USART2_IRQHandler()
{
  RosalynRx.USART2_IRQHandler();
}

extern "C" void USART3_4_IRQHandler()
{
  RosalynRx.USART3_4_IRQHandler();
}
